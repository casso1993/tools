#include "SLAMbagRecorder.h"

SLAMbagRecorder::SLAMbagRecorder() : is_recording_(false) {
    // 初始化ROS节点句柄
    home_dir_ = std::getenv("HOME");
    nh_.param("slam/bag_name", bag_name_, std::string("/BagRecorder/src/bagRecorder/bag/slam_data.bag"));
    nh_.param("slam/tf_topic", tf_topic_, std::string("/tf"));
    nh_.param("slam/odom_topic", odom_topic_, std::string("/odom"));
    nh_.param("slam/point_cloud_topic", point_cloud_topic_, std::string("/scan"));

    // 打开一个新的bag文件以写入模式
    bag_.open(home_dir_ + bag_name_, rosbag::bagmode::Write);

    // 设置订阅者
    sub_tf_ = nh_.subscribe(tf_topic_, 100, &SLAMbagRecorder::tfCallback, this);
    sub_odom_ = nh_.subscribe(odom_topic_, 10, &SLAMbagRecorder::odomCallback, this);
    sub_pointcloud_ = nh_.subscribe(point_cloud_topic_, 10, &SLAMbagRecorder::pointCloudCallback, this);
}

SLAMbagRecorder::~SLAMbagRecorder() {
    // 关闭bag文件
    if (bag_.isOpen()) {
        bag_.close();
        LOG(INFO) << "Bag file has been saved.";
    }
}

void SLAMbagRecorder::tfCallback(const tf2_msgs::TFMessageConstPtr &msg) {
    // 如果还没有开始录制，则开始录制并设置标志位
    if (!is_recording_) {
        LOG(INFO) << "Starting to record messages...";
        is_recording_ = true;
    }

    // 录制/tf消息到bag文件中
    bag_.write(tf_topic_, ros::Time::now(), msg);
}

void SLAMbagRecorder::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    if (is_recording_) {
        // 录制/odom消息到bag文件中
        bag_.write(odom_topic_, ros::Time::now(), msg);
    }
}

void SLAMbagRecorder::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (is_recording_) {
        // 录制/point_cloud消息到bag文件中
        bag_.write(point_cloud_topic_, ros::Time::now(), msg);
    }
}

int main(int argc, char **argv) {
    google::InitGoogleLogging("log");

    FLAGS_logtostderr = true;       // 将日志输出到标准错误流
    FLAGS_colorlogtostderr = true;  // 在控制台输出带有颜色的日志信息
    FLAGS_alsologtostderr = false;  // 同时将日志输出到文件和标准错误流
    FLAGS_log_dir = "./logs";       // 设置日志文件保存目录

    ros::init(argc, argv, "slam_data_bag_recorder");
    auto recorder = std::make_unique<SLAMbagRecorder>();

    LOG(WARNING) << "SLAM data bag recorder has been initialized.";

    // 进入循环，保持节点运行
    ros::spin();

    return 0;
}