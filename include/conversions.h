#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include "msg.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef union {
    float f;
    unsigned char bytes[sizeof(float)];
} FloatBytes;

float bytes_to_float(unsigned char *bytes);

namespace conversions {
    sensor_msgs::PointCloud2 fromXYZI(pcl::PointCloud<pcl::PointXYZINormal> pcl_pc);

    pcl::PointCloud<pcl::PointXYZINormal> toXYZI(sensor_msgs::PointCloud2 pc_msg);
}

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

#endif
