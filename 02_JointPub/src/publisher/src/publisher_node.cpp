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

void PCD_pub::publishCloud(ros::Publisher& pub, const pcl::PCLPointCloud2& cloud, const std::string& frame_id,
                           ros::Time& time) {
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = time;
    pub.publish(output);
}

ros::Time PCD_pub::stringToROSTime(const std::string& str) {
    long long milliseconds = std::stoll(str);
    ros::Time ros_time(milliseconds / 1000, milliseconds % 1000 * 1e6);
    return ros_time;
}

void PCD_pub::processing() {
    ros::Rate cloud_rate(PCD_pub_rate_);
    ThreadPool pool(8);  // 创建一个包含4个线程的线程池

    // 遍历所有文件
    for (const auto& file : files) {
        if (!ros::ok()) {
            LOG(WARNING) << "ROS program aborted  --PCD";
            return;
        }

        // 将任务加入线程池
        pool.enqueue([=]() {
            pcl::PCLPointCloud2 cloud;
            if (pcl::io::loadPCDFile(file, cloud) == -1) {
                LOG(ERROR) << "Couldn't read pcd file " << file.c_str();
                return;
            }

            std::filesystem::path absolute_path(file);
            std::string file_name = absolute_path.stem().string();
            ros::Time time = stringToROSTime(file_name);

            LOG(INFO) << "time: " << time.sec << "." << std::setw(9) << std::setfill('0') << time.nsec;
            publishCloud(cloud_pub, cloud, Laser_frame_id_, time);
        });

        cloud_rate.sleep();  // 控制发布频率

        if (&file == &files.back()) {
            LastElement = true;
            LOG(WARNING) << "=================>All pcd data has been published.";
            return;
        }
    }

    pool.~ThreadPool();  // 等待所有任务完成
}

bool PCD_pub::pcdReader() {
    boost::filesystem::path dir(homeDir + PCD_files_path_ + "/pcd");
    if (!boost::filesystem::exists(dir) || !boost::filesystem::is_directory(dir)) {
        LOG(ERROR) << "The directory of the pcd file does not exist or is not a directory: "
                   << (homeDir + PCD_files_path_ + "/pcd").c_str();
        return -1;
    }

    for (auto& entry : boost::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".pcd") files.push_back(entry.path().string());
    }

    // Sort by timestamp in the filename
    sort(files.begin(), files.end());
    return true;
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

bool IMU_pub::readBinFile(std::vector<STR_IMU>& data) {
    char pchBuffer[MAX_READ_LENGTH];
    int iRet = 0;
    STR_IMU* pstrTarget;
    int iDataLength = sizeof(STR_IMU);
    FILE* pFd_imu = fopen((homeDir + IMU_file_path_ + "/IMU.bin").c_str(), "rb");
    if (NULL == pFd_imu) {
        LOG(ERROR) << "The directory of the BIN file does not exist or is not a directory: "
                   << (homeDir + IMU_file_path_ + "/IMU.bin").c_str();
        return -1;
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
    return true;
}

void IMU_pub::publishIMUData(const std::vector<STR_IMU>& data) {
    ros::Rate imu_rate(IMU_pub_rate_);

    for (const auto& datum : data) {
        if (!ros::ok()) {
            LOG(WARNING) << "ROS program aborted  --IMU";
            return;
        }

        if (datum.ullSystemTime / 100000 > 2 * pow(10, 10)) {
            continue;
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

        imu_rate.sleep();

        if (&datum == &data.back()) {
            LastElement = true;
            LOG(WARNING) << "=================>All IMU data has been published.";
            return;
        }
    }
}

void node::run() {
    t1 = boost::thread([=]() { IMU->publishIMUData(data); });
    t2 = boost::thread([=]() { PCD->processing(); });

    while (t1.joinable() && t2.joinable()) {
        t1.join();
        t2.join();
    }
}

void node::readFile() {
    if (!IMU->readBinFile(data) || !PCD->pcdReader()) {
        return;
    }
    LOG(INFO) << "=================>Read file success.";
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

        if (main_node->getIMUStatus() || main_node->getPCDStatus()) {
            LOG(WARNING) << ">>>>>>>>>>   End publisher node.   <<<<<<<<<<";
            google::ShutdownGoogleLogging();
            break;
        }
    }
    return 0;
}