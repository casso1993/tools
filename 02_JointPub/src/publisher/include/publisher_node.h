#ifndef PUBLISHER_NODE_H
#define PUBLISHER_NODE_H
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sys/time.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>  // 控制t1, t2
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include "glog/logging.h"

#include <queue>
#include <variant>


using namespace std;

#define FILE_NAME_LENGTH 128
#define MAX_READ_LENGTH 50000
typedef unsigned char BYTE;
#define NEWIMU 1

#pragma pack(push, 1)

#if NEWIMU
typedef struct _STR_IMU {
    // BYTE               HEAD;
    unsigned long long ullSystemTime;
    double dLatitude;
    double dLongitude;
    float fAltitude;
    float fVn;
    float fVe;
    float fVu;
    float fRollAngle;
    float fPitchAngle;
    float fHeadingAngle;
    float fAccX;
    float fAccY;
    float fAccZ;
    float fGyroX;
    float fGyroY;
    float fGyroZ;
    char cSystemStatus;
    char GPS_Status;
    float fCanSpeed;
    float fReserve1;
    float fReserve2;
} STR_IMU, *PSTR_IMU;
#else

typedef struct s_imu {
    char reserve;
    double dLatitude;
    double dLongitude;
    float fAltitude;
    float fLateralSpeed;  // north
    float fLinearSpeed;   // east
    float fVerticalSpeed;
    float fRollAngle;
    float fPitchAngle;
    float fHeadingAngle;
    float fLinearAcceleration;
    float fLateralAcceleration;
    float fVerticalAcceleration;
    float fRollSpeed;
    float fPitchSpeed;
    float fHeadingSpeed;
    unsigned long long ullSystemTime;
    char cSystemStatus;
    char GPS_Status;
    float fOriLinearSpeed;
    float fSteeringAngle;
    float fSteeringSpeed;
} STR_IMU, *PSTR_IMU;

#endif
#pragma pack(pop)

const char *homeDir = std::getenv("HOME");

struct TimedEvent {
    ros::Time timestamp;
    enum EventType { IMU, PCD } type;
    std::variant<STR_IMU, std::string> data; // STR_IMU存储IMU数据，string存储PCD文件路径

    bool operator<(const TimedEvent& other) const {
        return timestamp > other.timestamp; // 优先队列按时间升序排列
    }
};

class IMU_pub {
   private:
    /* -----IMU parameter----- */
    ros::Publisher imu_pub;
    std::string IMU_topic_;
    std::string IMU_file_path_;
    std::string IMU_frame_id_;
    bool standard_timestamp_;

    /* -----gnss parameter----- */
    ros::Publisher gnss_pub;
    std::string gnss_topic_;
    std::string gnss_frame_id_;

    bool is_gnss_available_;

    /* -----odom parameter----- */
    // ros::Publisher odom_pub;
    bool is_path_available_;
    ros::Publisher path_pub;
    std::string path_topic_;
    std::string path_frame_id_;
    nav_msgs::Path path;

    ros::Time IMU_ros_time;
    
    double x_ = 0.0, y_ = 0.0, z_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;

    double dt = 1.0 / 125.0;
    int IMU_count = 0;

   public:
    double IMU_pub_rate_;

    IMU_pub();
    ~IMU_pub();

    std::vector<STR_IMU> readBinFile();
    void publishSingleIMU(const STR_IMU& datum);
};

class PCD_pub {
   private:
    void publishCloud(ros::Publisher &, const pcl::PCLPointCloud2 &, const std::string &, ros::Time &);

    std::vector<std::string> files;
    ros::Publisher cloud_pub;
    std::string PCD_files_path_;
    std::string Laser_topic_;
    std::string Laser_frame_id_;

    std::mutex mutex_;

   public:
    double PCD_pub_rate_;

    PCD_pub();
    ~PCD_pub();

    std::vector<std::pair<std::string, ros::Time>> pcdReader();
    void publishSinglePCD(const std::string& file);
    ros::Time stringToROSTime(const std::string &);
    std::string getFileName(const std::string&);
};

class node {
   private:
    boost::thread t1;
    boost::thread t2;
    std::unique_ptr<IMU_pub> IMU;
    std::unique_ptr<PCD_pub> PCD;

    std::vector<STR_IMU> data;

    std::priority_queue<TimedEvent> eventQueue;

    bool standard_timestamp_;

   public:
    node() : IMU(std::make_unique<IMU_pub>()), PCD(std::make_unique<PCD_pub>()){
        ros::NodeHandle nh;
        nh.param<bool>("IMU/standard_timestamp", standard_timestamp_, true);
    };
    ~node();

    void run();

    void readFile();
};

node::~node() {}

#endif  // PUBLISHER_NODE_H