#include <iostream>
#include <cmath>
#include <iomanip>
#include <yaml-cpp/yaml.h>

// 角度转弧度
// double deg2rad(double deg) {
//     return deg * M_PI / 180.0;
// }

// 弧度转角度
double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

int main() {
    std::string home_dir_ = getenv("HOME");
    YAML::Node config = YAML::LoadFile(home_dir_ + "/tools/12_ImuLidarTransform/config/config.yaml");
    // 已知参数：LiDAR到IMU的变换
    double flidar2imu_angle = config["angle"].as<double>();
    double flidar2imu_X = config["x"].as<double>();      // X方向平移
    double flidar2imu_Y = config["y"].as<double>();      // Y方向平移
    double flidar2imu_Z = 0.0;       // 假设Z方向平移为0
    
    std::cout << "已知参数(LiDAR到IMU的变换):" << std::endl;
    std::cout << "旋转角: " << flidar2imu_angle << std::endl;
    std::cout << "平移量: (" << flidar2imu_X << ", " << flidar2imu_Y << ", " << flidar2imu_Z << ")" << std::endl << std::endl;
    
    // 计算IMU到LiDAR的旋转角（原旋转角的相反数）
    double imu2lidar_angle = -flidar2imu_angle;
    // double imu2lidar_angle_rad = deg2rad(imu2lidar_angle);
    
    // 计算四元数 (w, x, y, z)
    double w = cos(imu2lidar_angle / 2.0);
    double x = 0.0;  // 绕Z轴旋转，x和y分量为0
    double y = 0.0;
    double z = sin(imu2lidar_angle / 2.0);
    
    // 计算旋转矩阵（绕Z轴）
    double cos_theta = cos(imu2lidar_angle);
    double sin_theta = sin(imu2lidar_angle);
    
    // 计算IMU到LiDAR的平移偏移
    // t_imu2lidar = -R_imu2lidar * t_lidar2imu
    double tx = -(cos_theta * flidar2imu_X - sin_theta * flidar2imu_Y);
    double ty = -(sin_theta * flidar2imu_X + cos_theta * flidar2imu_Y);
    double tz = -flidar2imu_Z;  // Z方向平移直接取反
    
    // 输出结果
    std::cout << "计算结果(IMU到LiDAR的变换):" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "旋转角: " << imu2lidar_angle << std::endl;
    std::cout << "四元数 (w, x, y, z): (" << w << ", " << x << ", " << y << ", " << z << ")" << std::endl;
    std::cout << "平移偏移 (tx, ty, tz): (" << tx << ", " << ty << ", " << tz << ")" << std::endl;
    
    return 0;
}
