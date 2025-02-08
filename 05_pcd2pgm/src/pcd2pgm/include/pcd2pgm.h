#ifndef PCD2PGM_H
#define PCD2PGM_H

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "glog/logging.h"

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

class pcd2pgm {
   private:
    ros::Publisher map_publisher;

    const std::string pcd_format = ".pcd";

    nav_msgs::OccupancyGrid map_msg;

    double min_height_threshold = 0.3;
    double max_height_threshold = 2.0;
    bool pass_through_flag = false;
    double map_resolution = 0.05;
    double radius_threshold = 0.1;
    int point_count_threshold = 10;

    std::string file_directory;
    std::string file_name;
    std::string pcd_file;
    std::string map_topic_name;

    CloudPtr cloud_after_pass_through{new PointCloudType};
    CloudPtr cloud_after_radius_filter{new PointCloudType};
    CloudPtr input_cloud{new PointCloudType};

   public:
    pcd2pgm(/* args */);
    ~pcd2pgm();

    void applyPassThroughFilter(double lower_limit, double upper_limit, bool invert);
    void applyRadiusOutlierFilter(const CloudPtr& cloud, double search_radius, int min_neighbors);
    void convertToOccupancyGrid(const CloudPtr cloud, nav_msgs::OccupancyGrid& grid_msg);
    void run();
};

#endif  // PCD2PGM_H