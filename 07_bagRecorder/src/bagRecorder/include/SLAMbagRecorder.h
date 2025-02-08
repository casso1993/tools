#ifndef SLAM_BAG_RECORDER_H
#define SLAM_BAG_RECORDER_H

#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <memory>

class SLAMbagRecorder {
   public:
    SLAMbagRecorder();
    ~SLAMbagRecorder();

   private:
    std::string home_dir_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_tf_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_pointcloud_;
    rosbag::Bag bag_;
    bool is_recording_;
    std::string bag_name_;
    std::string tf_topic_;
    std::string odom_topic_;
    std::string point_cloud_topic_;

    void tfCallback(const tf2_msgs::TFMessageConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

#endif  // SLAM_BAG_RECORDER_H
