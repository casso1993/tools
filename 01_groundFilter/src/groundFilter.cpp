#include "groundFilter.h"

groundFilter::groundFilter() {
    home_dir_ = getenv("HOME");
    if (home_dir_.empty()) {
        LOG(ERROR) << "Environment variable HOME is not set.";
    }

    const YAML::Node config = YAML::LoadFile(home_dir_ + "/tools/01_groundFilter/config/config.yaml");
    load_path_ = config["load_path"].as<std::string>();
    save_path_ = config["save_path"].as<std::string>();
    num_threads_ = config["num_threads"].as<size_t>();

    // APMF filter parameters
    APMF_available_ = config["APMF_available"].as<bool>();
    max_window_size_ = config["max_window_size"].as<float>();
    slope_ = config["slope"].as<float>();
    initial_distance_ = config["initial_distance"].as<float>();
    max_distance_ = config["max_distance"].as<float>();
    filter_cell_size_ = config["filter_cell_size"].as<float>();
    base_ = config["base"].as<float>();

    // CSF filter parameters
    CSF_available_ = config["CSF_available"].as<bool>();
    bSloopSmooth_ = config["bSloopSmooth"].as<bool>();
    time_step_ = config["time_step"].as<double>();
    class_threshold_ = config["class_threshold"].as<double>();
    cloth_resolution_ = config["cloth_resolution"].as<double>();
    rigidness_ = config["rigidness"].as<int>();

    // Height filter parameters
    height_filter_available_ = config["height_filter_available"].as<bool>();
    lidar_height_ = config["LiDAR_height_above_ground"].as<float>();
    filter_height_ = config["Filter_height_from_ground"].as<float>();
}

groundFilter::~groundFilter() {}

CloudPtr groundFilter::ApmfFilter(CloudPtr cloud) {
    LOG(INFO) << "APMF filter started.";
    CloudPtr output(new PointCloudType);
    pcl::ApproximateProgressiveMorphologicalFilter<PointType> APMF;
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    APMF.setInputCloud(cloud);
    APMF.setMaxWindowSize(max_window_size_);
    APMF.setSlope(slope_);
    APMF.setInitialDistance(initial_distance_);
    APMF.setMaxDistance(max_distance_);
    APMF.setCellSize(filter_cell_size_);
    APMF.setBase(base_);

    APMF.setExponential(true);  // 设置为true表示使用指数平滑滤波器
    APMF.extract(ground_indices->indices);

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(true);
    extract.filter(*output);
    return output;
}

CloudPtr groundFilter::HeightFilter(CloudPtr cloud) {
    LOG(INFO) << "Height filter started.";
    CloudPtr output(new PointCloudType);
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-lidar_height_, filter_height_ - lidar_height_);
    pass.setFilterLimitsNegative(false);  // 设置为false表示不删除输入点云中满足条件的点
    pass.filter(*output);
    return output;
}

CloudPtr groundFilter::CSFilter(CloudPtr cloud) {
    LOG(INFO) << "CSF filter started.";
    CloudPtr output(new PointCloudType);
    std::vector<csf::Point> csfPoints;
    for (const auto& point : cloud->points) {
        csfPoints.push_back(csf::Point(point.x, point.y, point.z));
    }
    CSF csfFilter;
    csfFilter.setPointCloud(csfPoints);
    csfFilter.params.bSloopSmooth = bSloopSmooth_;
    csfFilter.params.time_step = time_step_;
    csfFilter.params.class_threshold = class_threshold_;
    csfFilter.params.cloth_resolution = cloth_resolution_;
    csfFilter.params.rigidness = rigidness_;

    std::vector<int> groundIndexes, nonGroundIndexes;
    csfFilter.do_filtering(groundIndexes, nonGroundIndexes, true);
    for (int index : nonGroundIndexes) {
        output->points.push_back(cloud->points[index]);
    }
    return output;
}


void groundFilter::ProcessChunk(const std::pair<int, int>& range, CloudPtr input_cloud,
                                std::vector<CloudPtr>& results) {
    CloudPtr filtered_cloud(new PointCloudType);
    filtered_cloud->resize(range.second - range.first);

    for (int i = range.first; i < range.second; ++i) {
        filtered_cloud->points[i - range.first] = input_cloud->points[i];
    }

    if (APMF_available_) {
        filtered_cloud = ApmfFilter(filtered_cloud);
    }

    if (CSF_available_) {
        filtered_cloud = CSFilter(filtered_cloud);
    }

    if (height_filter_available_) {
        filtered_cloud = HeightFilter(filtered_cloud);
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    results.push_back(filtered_cloud);
}

void groundFilter::Run() {
    CloudPtr cloud(new PointCloudType);

    if (pcl::io::loadPCDFile<PointType>(home_dir_ + load_path_, *cloud) == -1) {
        LOG(ERROR) << "Couldn't read pcd file.";
        return;
    }

    LOG(INFO) << "Loaded point cloud with " << cloud->size() << " points.";

    size_t chunk_size = cloud->size() / num_threads_;
    std::vector<std::thread> threads;
    std::vector<std::pair<int, int>> ranges;
    std::vector<CloudPtr> processed_chunks;

    for (size_t i = 0; i < num_threads_; ++i) {
        int start = i * chunk_size;
        int end = (i == num_threads_ - 1) ? cloud->size() : start + chunk_size;
        auto range = std::make_pair(start, end);

        threads.emplace_back(
            [this, range, &cloud, &processed_chunks]() { this->ProcessChunk(range, cloud, processed_chunks); });
    }

    for (auto& th : threads) {
        if (th.joinable()) {
            th.join();
        }
    }

    CloudPtr final_cloud(new PointCloudType);

    for (const auto& chunk : processed_chunks) {
        *final_cloud += *chunk;
    }

    pcl::io::savePCDFileBinary(home_dir_ + save_path_, *final_cloud);

    LOG(INFO) << "Saved point cloud with " << final_cloud->size() << " points.";
    LOG(INFO) << "================> Done!";
}

int main() {
    google::InitGoogleLogging("log");

    FLAGS_logtostderr = true;       // 将日志输出到标准错误流
    FLAGS_colorlogtostderr = true;  // 在控制台输出带有颜色的日志信息
    FLAGS_alsologtostderr = false;  // 同时将日志输出到文件和标准错误流
    FLAGS_log_dir = "./logs";       // 设置日志文件保存目录

    auto filter = std::make_unique<groundFilter>();
    filter->Run();
    return 0;
}
