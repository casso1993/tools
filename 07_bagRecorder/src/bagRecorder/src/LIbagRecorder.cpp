#include "LIbagRecorder.h"

LIbagRecorder::LIbagRecorder() : is_recording_(false) {
    // 初始化ROS节点句柄
    home_dir_ = std::getenv("HOME");
    nh_.param("LI/bag_name", bag_name_, std::string("/BagRecorder/src/bagRecorder/bag/LI_data.bag"));
    nh_.param("LI/IMU_topic", imu_topic_, std::string("/imu_raw"));
    nh_.param("LI/LiDAR_topic", lidar_topic_, std::string("/points_raw"));
    nh_.param("LI/GNSS_topic", gnss_topic_, std::string("/gnss"));

    // 打开一个新的bag文件以写入模式
    bag_.open(home_dir_ + bag_name_, rosbag::bagmode::Write);

    // 设置订阅者
    sub_imu_ = nh_.subscribe(imu_topic_, 200, &LIbagRecorder::IMUCallback, this);
    sub_lidar_ = nh_.subscribe(lidar_topic_, 10, &LIbagRecorder::LiDARCallback, this);
    sub_gnss_ = nh_.subscribe(gnss_topic_, 200, &LIbagRecorder::GNSSCallback, this);
}

LIbagRecorder::~LIbagRecorder() {
    // 关闭bag文件
    if (bag_.isOpen()) {
        bag_.close();
        LOG(INFO) << "Bag file has been saved.";
    }
}

void LIbagRecorder::IMUCallback(const sensor_msgs::ImuConstPtr& msg) {
    // 如果还没有开始录制，则开始录制并设置标志位
    if (!is_recording_) {
        LOG(INFO) << "Starting to record messages...";
        is_recording_ = true;
    }

    // 录制/tf消息到bag文件中
    bag_.write(imu_topic_, ros::Time::now(), msg);
}

void LIbagRecorder::LiDARCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (is_recording_) {
        // 录制/odom消息到bag文件中
        bag_.write(lidar_topic_, ros::Time::now(), msg);
    }
}

void LIbagRecorder::GNSSCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (is_recording_) {
        bag_.write(gnss_topic_, ros::Time::now(), msg);
    }
}

int main(int argc, char** argv) {
    google::InitGoogleLogging("log");

    FLAGS_logtostderr = true;       // 将日志输出到标准错误流
    FLAGS_colorlogtostderr = true;  // 在控制台输出带有颜色的日志信息
    FLAGS_alsologtostderr = false;  // 同时将日志输出到文件和标准错误流
    FLAGS_log_dir = "./logs";       // 设置日志文件保存目录

    ros::init(argc, argv, "LiDAR_IMU_bag_recorder");
    auto recorder = std::make_unique<LIbagRecorder>();

    LOG(WARNING) << "LiDAR and IMU bag recorder has been initialized.";

    // 进入循环，保持节点运行
    ros::spin();

    return 0;
}
