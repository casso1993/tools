#ifndef LI_BAGRECORDER_H
#define LI_BAGRECORDER_H

#include <glog/logging.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <sensor_msgs/NavSatFix.h>

class LIbagRecorder {
   private:
    std::string home_dir_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_gnss_;
    
    rosbag::Bag bag_;
    bool is_recording_;

    std::string bag_name_;
    std::string imu_topic_;
    std::string lidar_topic_;
    std::string gnss_topic_;

    void IMUCallback(const sensor_msgs::ImuConstPtr& msg);
    void LiDARCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void GNSSCallback(const sensor_msgs::NavSatFixConstPtr& msg);

   public:
    LIbagRecorder(/* args */);
    ~LIbagRecorder();
};

#endif  // LI_BAGRECORDER_H
