#include "pcd2pgm.h"

// Apply a pass-through filter to the point cloud based on z-axis thresholds.
void pcd2pgm::applyPassThroughFilter(double lower_limit, double upper_limit, bool invert) {
    pcl::PassThrough<PointType> pass_through_filter;
    pass_through_filter.setInputCloud(input_cloud);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(lower_limit, upper_limit);
    pass_through_filter.setFilterLimitsNegative(invert);
    pass_through_filter.filter(*cloud_after_pass_through);

    pcl::io::savePCDFile<PointType>(file_directory + "map_filter.pcd", *cloud_after_pass_through);
    LOG(INFO) << "Number of points after pass-through filter: " << cloud_after_pass_through->points.size();
}

// Apply a radius outlier filter to remove noise from the point cloud.
void pcd2pgm::applyRadiusOutlierFilter(const CloudPtr& cloud, double search_radius, int min_neighbors) {
    pcl::RadiusOutlierRemoval<PointType> radius_outlier_filter;
    radius_outlier_filter.setInputCloud(cloud);
    radius_outlier_filter.setRadiusSearch(search_radius);
    radius_outlier_filter.setMinNeighborsInRadius(min_neighbors);
    radius_outlier_filter.filter(*cloud_after_radius_filter);

    pcl::io::savePCDFile<PointType>(file_directory + "map_radius_filter.pcd", *cloud_after_radius_filter);
    LOG(INFO) << "Number of points after radius outlier filter: " << cloud_after_radius_filter->points.size();
}

// Convert the filtered point cloud to an occupancy grid message.
void pcd2pgm::convertToOccupancyGrid(const CloudPtr cloud, nav_msgs::OccupancyGrid& grid_msg) {
    grid_msg.header.seq = 0;
    grid_msg.header.stamp = ros::Time::now();
    grid_msg.header.frame_id = "map";
    grid_msg.info.map_load_time = ros::Time::now();
    grid_msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max;
    double z_max_gray_rate = 0.05;
    double z_min_gray_rate = 0.95;
    double slope = (z_max_gray_rate - z_min_gray_rate) / (max_height_threshold - min_height_threshold);
    double intercept = (max_height_threshold * z_min_gray_rate - min_height_threshold * z_max_gray_rate) /
                       (max_height_threshold - min_height_threshold);

    if (cloud->points.empty()) {
        LOG(WARNING) << "Point cloud is empty!";
        return;
    }

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if (i == 0) {
            x_min = x_max = x;
            y_min = y_max = y;
        } else {
            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }
    }

    grid_msg.info.origin.position.x = x_min;
    grid_msg.info.origin.position.y = y_min;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.x = 0.0;
    grid_msg.info.origin.orientation.y = 0.0;
    grid_msg.info.origin.orientation.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.info.width = static_cast<uint32_t>((x_max - x_min) / map_resolution);
    grid_msg.info.height = static_cast<uint32_t>((y_max - y_min) / map_resolution);
    grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height, 0);

    for (size_t iter = 0; iter < cloud->points.size(); ++iter) {
        int i = static_cast<int>((cloud->points[iter].x - x_min) / map_resolution);
        int j = static_cast<int>((cloud->points[iter].y - y_min) / map_resolution);

        if (i >= 0 && i < grid_msg.info.width && j >= 0 && j < grid_msg.info.height) {
            grid_msg.data[j * grid_msg.info.width + i] = 100;
        }
    }
}

int main(int argc, char** argv) {
    google::InitGoogleLogging("log");

    FLAGS_logtostderr = true;       // 将日志输出到标准错误流
    FLAGS_colorlogtostderr = true;  // 在控制台输出带有颜色的日志信息
    FLAGS_alsologtostderr = false;  // 同时将日志输出到文件和标准错误流
    FLAGS_log_dir = "./logs";       // 设置日志文件保存目录

    ros::init(argc, argv, "pcl_filters");
    auto pgm = std::make_unique<pcd2pgm>();
    pgm->run();
    return 0;
}

void pcd2pgm::run() {
    ros::Rate loop_rate(1.0);

    if (pcl::io::loadPCDFile<PointType>(pcd_file, *input_cloud) == -1) {
        LOG(ERROR) << "Couldn't read file: " << pcd_file;
        return;
    }

    LOG(INFO) << "Initial number of points in point cloud: " << input_cloud->points.size();

    applyPassThroughFilter(min_height_threshold, max_height_threshold, pass_through_flag);
    applyRadiusOutlierFilter(cloud_after_pass_through, radius_threshold, point_count_threshold);
    convertToOccupancyGrid(cloud_after_radius_filter, map_msg);

    while (ros::ok()) {
        map_publisher.publish(map_msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
}

pcd2pgm::pcd2pgm() {
    std::string home_directory = getenv("HOME");
    ros::NodeHandle nh;

    nh.param<std::string>("file_directory", file_directory, std::string("/home/"));
    nh.param<std::string>("file_name", file_name, std::string("map"));
    pcd_file = home_directory + file_directory + file_name + pcd_format;

    nh.param<double>("min_height_threshold", min_height_threshold, 0.2);
    nh.param<double>("max_height_threshold", max_height_threshold, 2.0);
    nh.param<bool>("pass_through_flag", pass_through_flag, false);
    nh.param<double>("radius_threshold", radius_threshold, 0.5);
    nh.param<double>("map_resolution", map_resolution, 0.05);
    nh.param<int>("point_count_threshold", point_count_threshold, 10);
    nh.param<std::string>("map_topic_name", map_topic_name, std::string("map"));

    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);
}

pcd2pgm::~pcd2pgm() {}
