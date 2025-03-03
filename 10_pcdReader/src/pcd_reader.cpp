#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

std::string file;

struct PointXYZIR {
    PCL_ADD_POINT4D;  // This adds x, y, z, and a padded field to be 16-byte
                      // aligned.
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure proper alignment.
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                                      intensity)(uint16_t, ring, ring))
void readYamlParams(const std::string &filePath) {
    YAML::Node config = YAML::LoadFile(filePath);
    file = config["pcd_file"].as<std::string>();
}

int main() {
    // 假设你的YAML配置文件名为config.yaml
    std::string yamlFilePath = "/home/casso/tools/10_pcdReader/config/config.yaml";
    readYamlParams(yamlFilePath);

    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
    if (pcl::io::loadPCDFile<PointXYZIR>(file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file yourfile.pcd \n");
        return (-1);
    }
    if (!cloud->points.empty()) {
        std::cout << "Ring data of first point: " << cloud->points[700].ring << std::endl;
    } else {
        std::cout << "No points found in the cloud." << std::endl;
    }
    return 0;
}