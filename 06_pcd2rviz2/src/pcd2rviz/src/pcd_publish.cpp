#include "pcd_publish.h"

int main(int argc, char **argv)
{
    // google::InitGoogleLogging("log");

    // FLAGS_logtostderr = true;       // 将日志输出到标准错误流
    // FLAGS_colorlogtostderr = true;  // 在控制台输出带有颜色的日志信息
    // FLAGS_alsologtostderr = false;  // 同时将日志输出到文件和标准错误流
    // FLAGS_log_dir = "./logs";       // 设置日志文件保存目录

    rclcpp::init(argc, argv);
    auto publisher = std::make_shared<pcd_publish>();
    publisher->run();
    // google::ShutdownGoogleLogging();

    rclcpp::shutdown();
    return 0;
}

pcd_publish::pcd_publish() : Node("pcd_publisher")
{
    // 获取用户主目录
    home_directory = std::getenv("HOME");
    if (home_directory.empty())
    {
        // LOG(ERROR) << "Could not get HOME environment variable.";
        return;
    }

    point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_pcd", 1);

    this->declare_parameter<std::string>("load_path", "/tools/06_pcd2rviz2/src/pcd2rviz/pcd/transform.pcd");
    this->declare_parameter<std::string>("save_path", "/tools/06_pcd2rviz2/src/pcd2rviz/result/transform.pcd");
    this->declare_parameter<bool>("removal_available", false);
    this->declare_parameter<bool>("set_specific_height", false);
    this->declare_parameter<std::vector<double>>("specific_height", {0.0, 2.0});
    this->declare_parameter<double>("radius_search", 0.5);
    this->declare_parameter<int>("min_neighbors", 1);
    this->declare_parameter<bool>("height_filter_available", false);
    this->declare_parameter<double>("max_height", 10.0);
    this->declare_parameter<bool>("downsampling_available", false);
    this->declare_parameter<double>("downsampling_leaf_size", 0.5);
    this->declare_parameter<bool>("save_pcd_en", false);

    this->get_parameter("load_path", load_path);
    this->get_parameter("save_path", save_path);
    this->get_parameter("removal_available", removal_available);
    this->get_parameter("set_specific_height", set_specific_height);
    this->get_parameter("specific_height", specific_height);
    this->get_parameter("radius_search", radius_search);
    this->get_parameter("min_neighbors", min_neighbors);
    this->get_parameter("height_filter_available", height_filter_available);
    this->get_parameter("max_height", max_height);
    this->get_parameter("downsampling_available", downsampling_available);
    this->get_parameter("downsampling_leaf_size", downsampling_leaf_size);
    this->get_parameter("save_pcd_en", save_pcd_en);

    full_pcd_path = home_directory + load_path;
}

pcd_publish::~pcd_publish() {}

void pcd_publish::loadFile()
{
    // LOG(WARNING) << "Loading PCD file: " << full_pcd_path;

    // 加载PCD文件
    if (pcl::io::loadPCDFile<PointType>(full_pcd_path, *point_cloud) == -1)
    {
        // LOG(ERROR) << "Failed to read PCD file.";
        return;
    }
}

void pcd_publish::removeOutliers()
{
    CloudPtr filtered_cloud{new PointCloudType};
    CloudPtr unfiltered_cloud{new PointCloudType};

    if (set_specific_height)
    {
        for (const auto &point : *point_cloud)
        {
            if (point.z >= specific_height[0] && point.z <= specific_height[1])
            {
                filtered_cloud->points.push_back(point);
            }
            else
            {
                unfiltered_cloud->points.push_back(point);
            }
        }
        // LOG(INFO) << "Filtered point cloud with specific height: " << filtered_cloud->size() << " points.";
        // LOG(INFO) << "Unfiltered point cloud with specific height: " << unfiltered_cloud->size() << " points.";

        *point_cloud = *filtered_cloud;
    }

    pcl::RadiusOutlierRemoval<PointType> radius_outlier_removal;
    radius_outlier_removal.setInputCloud(point_cloud);
    radius_outlier_removal.setRadiusSearch(radius_search);          // 设置搜索半径
    radius_outlier_removal.setMinNeighborsInRadius(min_neighbors);  // 最小邻居数量
    radius_outlier_removal.filter(*point_cloud);

    *point_cloud += *unfiltered_cloud;
}

void pcd_publish::heightfilter()
{
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(point_cloud); // 设置输入点云
    pass.setFilterFieldName("z");    // 设置过滤字段为z轴，即高度
    pass.setFilterLimits(-10.0, max_height); // 设置高度限制，这里保留高度在0到10米之间的点
    // 如果想要删除小于0或大于10米的点，可以不启用以下反转范围的设置
    // pass.setFilterLimitsNegative(true); 

    // 执行滤波操作
    pass.filter(*point_cloud);
}

void pcd_publish::downsample()
{
    pcl::VoxelGrid<PointType> downsampling_filter;
    downsampling_filter.setInputCloud(point_cloud);
    downsampling_filter.setLeafSize(downsampling_leaf_size, downsampling_leaf_size,
                                    downsampling_leaf_size);  // 设置体素大小
    downsampling_filter.filter(*point_cloud);
}

void pcd_publish::run()
{
    loadFile();
    // LOG(INFO) << "Loaded point cloud with " << point_cloud->size() << " points.";

    // 移除离群点
    if (removal_available)
    {
        removeOutliers();
        // LOG(INFO) << "Removed outliers: " << point_cloud->size() << " points remaining.";
    }

    if (height_filter_available)
    {
        heightfilter();
        // LOG(INFO) << "Height filtered point cloud. " << point_cloud->size() << " points remaining.";
    }

    // 降采样
    if (downsampling_available)
    {
        downsample();
        // LOG(INFO) << "Downsampled point cloud. " << point_cloud->size() << " points remaining.";
    }

    // 将PCL点云转换为ROS消息
    sensor_msgs::msg::PointCloud2 ros_point_cloud_message;
    pcl::toROSMsg(*point_cloud, ros_point_cloud_message);

    // 设置坐标系ID
    ros_point_cloud_message.header.frame_id = "map";
    ros_point_cloud_message.header.stamp = this->get_clock()->now();

    // 保存PCD文件
    if (downsampling_available && save_pcd_en)
    {
        std::string save_file = home_directory + save_path;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(save_file, *point_cloud);
        // LOG(WARNING) << "Saving PCD file: " << save_file;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    point_cloud_pub->publish(ros_point_cloud_message);

    rclcpp::spin_some(this->get_node_base_interface());
}