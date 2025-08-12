#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// 添加Boost线程库头文件
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

int main(int argc, char**argv) {
    // 检查输入参数
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <点云文件1.pcd> <点云文件2.pcd>" << std::endl;
        return -1;
    }

    // 定义点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud1) == -1) {
        std::cerr << "无法读取文件: " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "成功读取点云1: " << cloud1->width * cloud1->height << " 个点" << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud2) == -1) {
        std::cerr << "无法读取文件: " << argv[2] << std::endl;
        return -1;
    }
    std::cout << "成功读取点云2: " << cloud2->width * cloud2->height << " 个点" << std::endl;

    // 创建可视化对象
    pcl::visualization::PCLVisualizer viewer("点云可视化");
    
    // 设置背景颜色为黑色
    viewer.setBackgroundColor(0, 0, 0);
    
    // 添加第一个点云（红色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud1, single_color1, "cloud1");
    
    // 添加第二个点云（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud2, single_color2, "cloud2");
    
    // 设置点的大小
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
    
    // 添加坐标系
    viewer.addCoordinateSystem(1.0);
    
    // 可视化循环
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        // 使用完整的命名空间访问Boost的this_thread
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}