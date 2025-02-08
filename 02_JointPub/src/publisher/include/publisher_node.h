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

class IMU_pub {
   private:
    /* -----IMU parameter----- */
    ros::Publisher imu_pub;
    std::string IMU_topic_;
    std::string IMU_file_path_;
    std::string IMU_frame_id_;
    double IMU_pub_rate_;

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

    bool LastElement = false;

    double x_ = 0.0, y_ = 0.0, z_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;

    double dt = 1.0 / 125.0;
    int IMU_count = 0;

   public:
    IMU_pub();
    ~IMU_pub();

    bool readBinFile(std::vector<STR_IMU> &);
    void publishIMUData(const std::vector<STR_IMU> &);

    inline bool isLastElement() { return LastElement; };
};

class PCD_pub {
   private:
    void publishCloud(ros::Publisher &, const pcl::PCLPointCloud2 &, const std::string &, ros::Time &);

    std::vector<std::string> files;
    ros::Publisher cloud_pub;
    std::string PCD_files_path_;
    std::string Laser_topic_;
    std::string Laser_frame_id_;
    double PCD_pub_rate_;

    bool LastElement = false;
    std::mutex mutex_;

   public:
    PCD_pub();
    ~PCD_pub();

    bool pcdReader();
    void processing();
    inline bool isLastElement() { return LastElement; };
    ros::Time stringToROSTime(const std::string &);
};

class node {
   private:
    boost::thread t1;
    boost::thread t2;
    std::unique_ptr<IMU_pub> IMU;
    std::unique_ptr<PCD_pub> PCD;

    std::vector<STR_IMU> data;

   public:
    node() : IMU(std::make_unique<IMU_pub>()), PCD(std::make_unique<PCD_pub>()){};
    ~node();

    void run();

    void readFile();

    inline bool getIMUStatus() const { return IMU->isLastElement(); }
    inline bool getPCDStatus() const { return PCD->isLastElement(); }
};

node::~node() {}

class ThreadPool {
   public:
    ThreadPool(size_t threads) : stop(false) {
        for (size_t i = 0; i < threads; ++i)
            workers.emplace_back([this] {
                for (;;) {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty()) return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }

                    task();
                }
            });
    }

    template <class F, class... Args>
    auto enqueue(F &&f, Args &&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);

            // don't allow enqueueing after stopping the pool
            if (stop) throw std::runtime_error("enqueue on stopped ThreadPool");

            tasks.emplace([task]() { (*task)(); });
        }
        condition.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : workers) worker.join();
    }

   private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;

    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};
#endif  // PUBLISHER_NODE_H