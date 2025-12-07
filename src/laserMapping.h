#ifndef INTERFACE_H
#define INTERFACE_H

#include <optional>
#include "msg.h"

#if defined(_WIN32) || defined(__CYGWIN__)
  #ifdef _MSC_VER
    #define FASTLIO_EXPORT __declspec(dllexport)
  #else
    #define FASTLIO_EXPORT __attribute__((dllexport))
  #endif
#else
  #define FASTLIO_EXPORT __attribute__((visibility("default")))
#endif

struct LidarOutput {
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudFull;
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudFull_body;
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudEffect;
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudMap;
    std::optional<nav_msgs::Odometry> pubOdomAftMapped;
    std::optional<nav_msgs::Path> pubPath;
};

FASTLIO_EXPORT void init();
FASTLIO_EXPORT std::optional<LidarOutput> run();
FASTLIO_EXPORT void save_map();
FASTLIO_EXPORT void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
FASTLIO_EXPORT void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);

#endif
