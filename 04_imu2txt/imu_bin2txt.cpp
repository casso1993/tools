#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

using namespace std;

#define FILE_NAME_LENGTH 128
#define MAX_READ_LENGTH 50000
typedef unsigned char BYTE;
#define NEWIMU 1
#define PI 3.1415926535897932384626433832795

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

bool column_name = false;

int main(int argc, char **argv) {
    google::InitGoogleLogging("log");

    FLAGS_logtostderr = true;       // 将日志输出到标准错误流
    FLAGS_colorlogtostderr = true;  // 在控制台输出带有颜色的日志信息
    FLAGS_alsologtostderr = false;  // 同时将日志输出到文件和标准错误流
    FLAGS_log_dir = "./logs";       // 设置日志文件保存目录

    const YAML::Node config = YAML::LoadFile("../config/config.yaml");
    int output_type_ = config["output_type"].as<int>();

    if (argc < 3) {
        LOG(ERROR) << "usage: imu_bin2txt bin_file txt_file" << endl;
        return -1;
    }
    char pchBuffer[MAX_READ_LENGTH];
    int iRet = 0;
    STR_IMU *pstrTarget;
    int iDataLength = sizeof(STR_IMU);
    std::string strFilePath(argv[1]);
    std::string strTxtPath(argv[2]);
    FILE *pFd_imu = fopen(strFilePath.c_str(), "rb");
    FILE *pSaveFile = fopen(strTxtPath.c_str(), "w");
    if (NULL == pFd_imu || NULL == pSaveFile) {
        LOG(ERROR) << "=====> Cannot Open file " << endl;
        return -1;
    }
    LOG(INFO) << "size of IMU: " << sizeof(STR_IMU) << std::endl;
    while (!feof(pFd_imu)) {
#if 0
        iRet= fread(pchBuffer, 1, iDataLength + 9, pFd_imu);
        if(iRet != iDataLength + 9)
        {
            if(!feof(pFd_imu))
            {
                printf("<%s> Read data fail, returned len %u, expected %u, EOF = %d.\n", 
                strFilePath.c_str(), iRet, iDataLength, EOF);
            }
            break;
        }
		pstrTarget = (STR_IMU *)(pchBuffer+8);
#else
        int tt = fgetc(pFd_imu);
        if (tt != 0x0E) {
            continue;
        }
        iRet = fread(pchBuffer, 1, iDataLength, pFd_imu);
        pstrTarget = (STR_IMU *)pchBuffer;
#endif
        // pstrTarget = (STR_IMU *)pchBuffer;
        double lat = pstrTarget->dLatitude;
        double lon = pstrTarget->dLongitude;
        double alti = pstrTarget->fAltitude;
        double roll = pstrTarget->fRollAngle;
        double pitch = pstrTarget->fPitchAngle;
        double yaw = pstrTarget->fHeadingAngle;

#if NEWIMU
        double Vn = pstrTarget->fVn;
        double Ve = pstrTarget->fVe;
        double Vu = pstrTarget->fVu;
        float fAccX = pstrTarget->fAccX;
        float fAccY = pstrTarget->fAccY;
        float fAccZ = pstrTarget->fAccZ;
        float fGyroX = pstrTarget->fGyroX;
        float fGyroY = pstrTarget->fGyroY;
        float fGyroZ = pstrTarget->fGyroZ;
#else
        double lateralSpeed = pstrTarget->fLateralSpeed;
        double linearSpeed = pstrTarget->fLinearSpeed;
        double verticalSpeed = pstrTarget->fVerticalSpeed;
#endif
        int gps_state = pstrTarget->GPS_Status;
        int sys_state = pstrTarget->cSystemStatus;

        float fCanSpeed = pstrTarget->fCanSpeed;
        /*
            float fLateralSpeed;
            float fLinearSpeed;
            float fVerticalSpeed;
        */
        // unsigned long long ulltime = pstrTarget->ullSystemTime;
        unsigned long long ulltime = pstrTarget->ullSystemTime / 100;
        LOG(INFO) << "gps_state:" << gps_state << "  sys_state:" << sys_state << endl;
        static unsigned long long ullLasttime = ulltime;
        int iTimeDiff = ulltime - ullLasttime;
#if NEWIMU
        switch (output_type_) {
            case 1:
                if (gps_state == 4) {
                    fprintf(pSaveFile, "%lld %f %f %f %d %d %f %f %f %f %f %f %f\n", ulltime, Vn, Ve, Vu, gps_state,
                            sys_state, fAccX, fAccY, fAccZ, fGyroX, fGyroY, fGyroZ, fCanSpeed);
                }
                break;

            case 2:
                fprintf(pSaveFile, "%.3f %.10f %.10f %f %f %f %f %f %f %f %d %d %f %d\n", ulltime / 1000.0, lon, lat,
                        alti, roll, pitch, yaw, Vn, Ve, Vu, gps_state, sys_state, fCanSpeed, iTimeDiff);
                break;

            case 3:
                fprintf(pSaveFile, "%lld.%03lld %f %f %f %f %f %f \n", ulltime / 1000, ulltime % 1000, fAccX, fAccY,
                        fAccZ, fGyroX, fGyroY, fGyroZ);
                break;

            case 4:
                if (!column_name) {
                    fprintf(pSaveFile, "gps_time,x,y,z,ve(m/s),vn(m/s),vu(m/s),roll(rad),pitch(rad),yaw(rad)\n");
                    column_name = true;
                }

                if (ulltime > 2e12) break;

                time_t seconds = ulltime / 1000;
                int milliseconds = ulltime % 1000;
                struct tm *timeinfo = gmtime(&seconds);

                fprintf(pSaveFile, "%04d-%02d-%02d-%02d-%02d-%02d-%03d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour,
                        timeinfo->tm_min, timeinfo->tm_sec, milliseconds, lon * 1e5, lat * 1e5, alti, Ve, Vn, Vu,
                        roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
                break;
        }

#else

        fprintf(pSaveFile, "%lld %f %f %f %f %f %f %f %f %f %d %d\n", ulltime, lat, lon, alti, roll, pitch, yaw,
                lateralSpeed, linearSpeed, verticalSpeed, gps_state, sys_state);
#endif
        // char ch = getchar();
        ullLasttime = ulltime;
    }
    std::fclose(pFd_imu);
    std::fclose(pSaveFile);
    cout << "finish!" << endl;
    return 0;
}
