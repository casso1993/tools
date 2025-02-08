#ifndef GROUND_FILTER_H
#define GROUND_FILTER_H

#include <glog/logging.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <mutex>
#include <thread>
#include <vector>
#include "../CSF/src/CSF.h"
#include "yaml-cpp/yaml.h"

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

class groundFilter {
   private:
    CloudPtr CSFilter(CloudPtr);
    CloudPtr ApmfFilter(CloudPtr);
    CloudPtr HeightFilter(CloudPtr);
    void ProcessChunk(const std::pair<int, int>&, CloudPtr, std::vector<CloudPtr>&);

    // Mutex for protecting shared resources
    std::mutex cloud_mutex_;

    std::string load_path_;
    std::string save_path_;
    size_t num_threads_;
    std::string home_dir_;

    /* Parameters */
    // APMF filter
    bool APMF_available_ = false;
    float max_window_size_;
    float slope_;
    float initial_distance_;
    float max_distance_;
    float filter_cell_size_;
    float base_;

    // CSF filter
    bool CSF_available_ = false;
    bool bSloopSmooth_;
    double time_step_;
    double class_threshold_;
    double cloth_resolution_;
    int rigidness_;

    // Height filter
    bool height_filter_available_ = false;
    float lidar_height_;
    float filter_height_;

   public:
    groundFilter();
    ~groundFilter();
    void Run();
};
#endif
