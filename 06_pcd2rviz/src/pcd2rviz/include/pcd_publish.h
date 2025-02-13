#ifndef PCD_PUBLISH_H
#define PCD_PUBLISH_H

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "glog/logging.h"

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

class pcd_publish {
   private:
    std::string home_directory;
    ros::Publisher point_cloud_pub;

    std::string load_path;
    std::string save_path;
    bool removal_available;
    bool set_specific_height;
    std::vector<double> specific_height;
    double radius_search;
    int min_neighbors;
    bool height_filter_available;
    double max_height;
    bool downsampling_available;
    double downsampling_leaf_size;
    bool save_pcd_en;

    std::string full_pcd_path;
    CloudPtr point_cloud{new PointCloudType};

   public:
    pcd_publish(/* args */);
    ~pcd_publish();

    void run();
    void loadFile();
    void removeOutliers();
    void heightfilter();
    void downsample();
};

#endif