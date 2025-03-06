#include "publisher_node.h"

#define PI 3.141592

PCD_pub::PCD_pub() {
    ros::NodeHandle nh;

    nh.param<std::string>("PCD_files_path", PCD_files_path_, "/Documents/RSdata/mh2");
    nh.param<std::string>("LiDAR/topic", Laser_topic_, "/points_raw");
    nh.param<std::string>("LiDAR/frame_id", Laser_frame_id_, "laser");
    nh.param<double>("LiDAR/pub_rate", PCD_pub_rate_, 1.0);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(Laser_topic_, 20);
}

PCD_pub::~PCD_pub() {}

std::string PCD_pub::getFileName(const std::string& filePath) {
    std::filesystem::path path(filePath);
    return path.filename().string();
}

ros::Time PCD_pub::stringToROSTime(const std::string& str) {
    long long milliseconds = std::stoll(str);
    ros::Time ros_time(milliseconds / 1000, milliseconds % 1000 * 1e6);
    return ros_time;
}

std::vector<std::pair<std::string, ros::Time>> PCD_pub::pcdReader() {
    std::vector<std::pair<std::string, ros::Time>> filesWithTime;
    std::vector<std::string> files;
    boost::filesystem::path dir(homeDir + PCD_files_path_ + "/pcd");
    if (!boost::filesystem::exists(dir) || !boost::filesystem::is_directory(dir)) {
        LOG(ERROR) << "The directory of the pcd file does not exist or is not a directory: "
                   << (homeDir + PCD_files_path_ + "/pcd").c_str();
        return filesWithTime;
    }

    for (auto& entry : boost::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".pcd") files.push_back(entry.path().string());
    }
    for (auto& file : files) {
        std::string filename = getFileName(file);
        ros::Time time = stringToROSTime(filename);
        filesWithTime.emplace_back(file, time);
    }
    // 按时间排序
    std::sort(filesWithTime.begin(), filesWithTime.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });
    return filesWithTime;
}

/** 原来的实现方法
void PCD_pub::publishSinglePCD(const std::string& file) {
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(file, cloud) == -1) {
        LOG(ERROR) << "Couldn't read pcd file " << file;
        return;
    }
    ros::Time time = stringToROSTime(getFileName(file));
    publishCloud(cloud_pub, cloud, Laser_frame_id_, time);
}

void PCD_pub::publishCloud(ros::Publisher& pub, const pcl::PCLPointCloud2& cloud, const std::string& frame_id,
                           ros::Time& time) {
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = time;
    pub.publish(output);
}
*/

/** 修改后的实现方法,失败 
void PCD_pub::publishSinglePCD(const std::string& pcd_path) {
    std::ifstream ifs(pcd_path);
    if (!ifs.is_open()) {
        LOG(ERROR) << "Failed to open file: " << pcd_path;
        return;
    }
    std::string line;
    // 读取并忽略头部信息
    for (int i = 0; i < 11; ++i) {  // 跳过前11行的头信息
        if (!std::getline(ifs, line)) {
            LOG(ERROR) << "Invalid PCD file format";
            return;
        }
    }

    PointCloudMsg cloud;
    cloud.points.clear();  // 清空可能已有的点云数据

    // 开始处理点数据
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        PointXYZIRT point;
        if (!(iss >> point.x >> point.y >> point.z >> point.intensity >> point.ring >> point.timestamp)) {
            LOG(ERROR) << "Error reading point data";
            continue;
        }
        cloud.points.push_back(point);
    }
    cloud_pub.publish(toRosMsg(cloud, Laser_frame_id_));
    std::cout << "9" << std::endl;
}

inline sensor_msgs::PointCloud2 PCD_pub::toRosMsg(const PointCloudMsg& rs_msg, const std::string& frame_id) {
    sensor_msgs::PointCloud2 ros_msg;

    int fields = 6;
    ros_msg.fields.clear();
    ros_msg.fields.reserve(fields);

    ros_msg.width = rs_msg.height;  // exchange width and height to be compatible with pcl::PointCloud<>
    ros_msg.height = rs_msg.width;

    int offset = 0;
    offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
    offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);

    ros_msg.point_step = offset;
    ros_msg.row_step = ros_msg.width * ros_msg.point_step;
    ros_msg.is_dense = rs_msg.is_dense;
    ros_msg.data.resize(ros_msg.point_step * ros_msg.width * ros_msg.height);

    sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
    sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");

    std::cout << "4" << std::endl;

    for (size_t i = 0; i < rs_msg.points.size();
         ++i, ++iter_x_, ++iter_y_, ++iter_z_, ++iter_intensity_, ++iter_ring_, ++iter_timestamp_) {
        const PointXYZIRT& point_ = rs_msg.points[i];

        // std::cout << "x: " << point.x << std::endl;
        // std::cout << "y: " << point.y << std::endl;
        // std::cout << "z: " << point.z << std::endl;
        // std::cout << "i: " << point.intensity << std::endl;
        // std::cout << "r: " << point.ring << std::endl;
        // std::cout << "t: " << point.timestamp << std::endl;

        std::cout << "*iter_x_: " << *iter_x_ << std::endl;

        *iter_x_ = point_.x;
        *iter_y_ = point_.y;
        *iter_z_ = point_.z;
        *iter_intensity_ = point_.intensity;
        *iter_ring_ = point_.ring;
        *iter_timestamp_ = point_.timestamp;
    }

    ros_msg.header.seq = rs_msg.seq;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
    ros_msg.header.frame_id = frame_id;
    std::cout << "8" << std::endl;
    return ros_msg;
}

// void PCD_pub::publishCloud(ros::Publisher& pub, const PointXYZIRT& cloud, const std::string& frame_id,
//                            ros::Time& time) {
//     sensor_msgs::PointCloud2 output;
//     pcl_conversions::fromPCL(cloud, output);
//     output.header.frame_id = frame_id;
//     output.header.stamp = time;
//     pub.publish(output);
// }
*/

void PCD_pub::publishSinglePCD(const std::string& pcd_path) {
    // 加载PCD文件
    pcl::PointCloud<PointXYZIRT>::Ptr cloud(new pcl::PointCloud<PointXYZIRT>);
    if (pcl::io::loadPCDFile<PointXYZIRT>(pcd_path, *cloud) == -1) {
        ROS_ERROR_STREAM("Failed to load PCD file: " << pcd_path);
        return;
    }

    // 转换为ROS消息
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = Laser_frame_id_;

    // 手动修正字段描述（PCL转换可能不保留原始顺序）
    msg.fields.clear();
    
    sensor_msgs::PointField field;
    field.name = "x"; field.offset = 0; 
    field.datatype = sensor_msgs::PointField::FLOAT32; field.count = 1;
    msg.fields.push_back(field);
    
    field.name = "y"; field.offset = 4;
    msg.fields.push_back(field);
    
    field.name = "z"; field.offset = 8;
    msg.fields.push_back(field);
    
    field.name = "intensity"; field.offset = 12;
    field.datatype = sensor_msgs::PointField::UINT8;
    msg.fields.push_back(field);
    
    field.name = "ring"; field.offset = 13;
    field.datatype = sensor_msgs::PointField::UINT16;
    msg.fields.push_back(field);
    
    field.name = "timestamp"; field.offset = 15;
    field.datatype = sensor_msgs::PointField::FLOAT64;
    msg.fields.push_back(field);

    // 设置点云参数
    msg.point_step = sizeof(PointXYZIRT);
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;

    ros::Time time = stringToROSTime(getFileName(pcd_path));
    msg.header.stamp = time;
    cloud_pub.publish(msg);
}

IMU_pub::IMU_pub() {
    ros::NodeHandle nh;

    nh.param<std::string>("IMU_file_path", IMU_file_path_, "/Documents/RSdata/mh2");
    nh.param<std::string>("IMU/topic", IMU_topic_, "imu_raw");
    nh.param<std::string>("IMU/frame_id", IMU_frame_id_, "imu_link");
    nh.param<double>("IMU/pub_rate", IMU_pub_rate_, 120.0);

    nh.param<bool>("is_path_available", is_path_available_, false);
    nh.param<std::string>("path/topic", path_topic_, "imu_path");
    nh.param<std::string>("path/frame_id", path_frame_id_, "odom");

    nh.param<bool>("is_GNSS_available", is_gnss_available_, false);
    nh.param<std::string>("GNSS/topic", gnss_topic_, "gnss_raw");
    nh.param<std::string>("GNSS/frame_id", gnss_frame_id_, "gnss_link");

    imu_pub = nh.advertise<sensor_msgs::Imu>(IMU_topic_, 200);
    gnss_pub = nh.advertise<sensor_msgs::NavSatFix>(gnss_topic_, 200);

    path_pub = nh.advertise<nav_msgs::Path>(path_topic_, 200);

    path.header.frame_id = path_frame_id_;
    path.header.stamp = ros::Time::now();
}

IMU_pub::~IMU_pub() {}

std::vector<STR_IMU> IMU_pub::readBinFile() {
    std::vector<STR_IMU> data;
    char pchBuffer[MAX_READ_LENGTH];
    int iRet = 0;
    STR_IMU* pstrTarget;
    int iDataLength = sizeof(STR_IMU);
    FILE* pFd_imu = fopen((homeDir + IMU_file_path_ + "/IMU.bin").c_str(), "rb");
    if (NULL == pFd_imu) {
        LOG(ERROR) << "The directory of the BIN file does not exist or is not a directory: "
                   << (homeDir + IMU_file_path_ + "/IMU.bin").c_str();
        return data;
    }
    while (!feof(pFd_imu)) {
#if 0
        iRet= fread(pchBuffer, 1, iDataLength + 9, pFd_imu);
        if(iRet != iDataLength + 9)
        {
            if(!feof(pFd_imu))
            {
                printf("<%s> Read data fail, returned len %u, expected %u, EOF = %d.\n", 
                (homeDir + IMU_file_path_ + "/IMU.bin").c_str(), iRet, iDataLength, EOF);
            }
            break;
        }
		pstrTarget = (STR_IMU *)(pchBuffer+8);
        data.push_back(pstrTarget);
#else
        int tt = fgetc(pFd_imu);
        if (tt != 0x0E) {
            continue;
        }
        iRet = fread(pchBuffer, 1, iDataLength, pFd_imu);
        pstrTarget = (STR_IMU*)pchBuffer;
        data.push_back(*pstrTarget);
#endif
    }
    return data;
}

void IMU_pub::publishSingleIMU(const STR_IMU& datum) {
    if (datum.ullSystemTime / 100000 > 2 * pow(10, 10)) {
        return;
    }

    sensor_msgs::Imu msg;

    ros::Time IMU_ros_time(datum.ullSystemTime / 100000, datum.ullSystemTime % 100000 * 10000);
    msg.header.stamp = IMU_ros_time;
    msg.header.frame_id = IMU_frame_id_;

    geometry_msgs::Quaternion quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(datum.fRollAngle, datum.fPitchAngle, datum.fHeadingAngle);
    msg.orientation = quaternion;

    msg.orientation_covariance[0] = -1;  // 表示没有方向信息

    msg.angular_velocity.x = datum.fGyroX * PI / 180.0;
    msg.angular_velocity.y = datum.fGyroY * PI / 180.0;
    msg.angular_velocity.z = datum.fGyroZ * PI / 180.0;

    msg.linear_acceleration.x = datum.fAccX;
    msg.linear_acceleration.y = datum.fAccY;
    msg.linear_acceleration.z = datum.fAccZ;

    imu_pub.publish(msg);

    if (is_gnss_available_) {
        sensor_msgs::NavSatFix gnss_msg;
        gnss_msg.header.stamp = IMU_ros_time;
        gnss_msg.header.frame_id = gnss_frame_id_;
        gnss_msg.latitude = datum.dLatitude;
        gnss_msg.longitude = datum.dLongitude;
        gnss_msg.altitude = datum.fAltitude;
        gnss_msg.status.status = datum.cSystemStatus;
        gnss_pub.publish(gnss_msg);
    }

    /* 发布的是path */
    if (is_path_available_) {
        geometry_msgs::PoseStamped pose;

        double dt = 1 / IMU_pub_rate_;
        vx_ += datum.fAccX * dt;
        vy_ += datum.fAccY * dt;
        vz_ = 0;
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        z_ = 0;

        roll_ += datum.fGyroX * dt * PI / 180.0;
        pitch_ += datum.fGyroY * dt * PI / 180.0;
        yaw_ += datum.fGyroZ * dt * PI / 180.0;
        LOG(INFO) << " yaw: " << yaw_ * 180.0 / PI;

        tf::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = path_frame_id_;
        pose.pose.position.x = x_;
        pose.pose.position.y = y_;
        pose.pose.position.z = z_;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path.poses.push_back(pose);
        if (IMU_count % static_cast<int>(IMU_pub_rate_) == 0) {
            path_pub.publish(path);
        }

        IMU_count++;
    }
}

void node::readFile() {
    auto imuData = IMU->readBinFile();
    auto pcdFiles = PCD->pcdReader();

    // 填充IMU事件
    for (const auto& datum : imuData) {
        TimedEvent event;
        event.type = TimedEvent::IMU;
        event.data = datum;
        event.timestamp = ros::Time(datum.ullSystemTime / 100000, (datum.ullSystemTime % 100000) * 10000);
        eventQueue.push(event);
    }

    // 填充PCD事件
    for (const auto& file : pcdFiles) {
        TimedEvent event;
        event.type = TimedEvent::PCD;
        event.data = file.first;
        event.timestamp = file.second;
        eventQueue.push(event);
    }
}

void node::run() {
    ros::Rate rate(std::max(PCD->PCD_pub_rate_, IMU->IMU_pub_rate_));
    while (!eventQueue.empty() && ros::ok()) {
        TimedEvent event = eventQueue.top();
        eventQueue.pop();

        if (event.type == TimedEvent::IMU) {
            STR_IMU imuData = std::get<STR_IMU>(event.data);
            IMU->publishSingleIMU(imuData);
        } else {
            std::string pcdPath = std::get<std::string>(event.data);
            PCD->publishSinglePCD(pcdPath);
        }
        LOG(INFO) << "Publishing " << (event.type == TimedEvent::IMU ? "IMU" : "PCD")
                  << " at time: " << event.timestamp;

        rate.sleep();
    }

    LOG(WARNING) << "=================>All data published.";
    ros::shutdown();
}

int main(int argc, char** argv) {
    google::InitGoogleLogging("log");

    FLAGS_logtostderr = true;       // 将日志输出到标准错误流
    FLAGS_colorlogtostderr = true;  // 在控制台输出带有颜色的日志信息
    FLAGS_alsologtostderr = false;  // 同时将日志输出到文件和标准错误流
    FLAGS_log_dir = "./logs";       // 设置日志文件保存目录

    LOG(WARNING) << ">>>>>>>>>>   Start publisher node.   <<<<<<<<<<";

    ros::init(argc, argv, "publisher_node");
    auto main_node = std::make_unique<node>();

    main_node->readFile();

    while (ros::ok()) {
        main_node->run();
    }
    return 0;
}