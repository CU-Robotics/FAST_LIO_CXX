#include "conversions.h"

float bytes_to_float(unsigned char *bytes){
    FloatBytes data;
    data.bytes[0] = *(bytes);
    data.bytes[1] = *(bytes+1);
    data.bytes[2] = *(bytes+2);
    data.bytes[3] = *(bytes+3);
    return data.f;
}

namespace conversions {
    sensor_msgs::PointCloud2 fromXYZI(pcl::PointCloud<pcl::PointXYZINormal> pcl_pc) {
        const uint32_t num_points = sizeof(pcl_pc.points) / sizeof(pcl_pc.points[0]);
        std::vector<uint8_t> point_data;
        for (pcl::PointXYZINormal point: pcl_pc.points) {
            FloatBytes x, y, z, intensity;
            x.f = point.x;
            y.f = point.y;
            z.f = point.z;
            intensity.f = point.intensity;
            point_data.push_back(static_cast<uint8_t>(*x.bytes));
            point_data.push_back(static_cast<uint8_t>(*y.bytes));
            point_data.push_back(static_cast<uint8_t>(*z.bytes));
            point_data.push_back(static_cast<uint8_t>(*intensity.bytes));
        }
        return {
            {0},
            1,
            num_points,
            false,
            4 * sizeof(float),
            4 * sizeof(float) * num_points,
            false,
            point_data,
            {
                {"x", 0, 7}, // 7: FLOAT32
                {"y", 4, 7}, // 7: FLOAT32
                {"z", 8, 7}, // 7: FLOAT32
                {"intensity", 12, 7}, // 7: FLOAT32
            }
        };
    };

    pcl::PointCloud<pcl::PointXYZINormal> toXYZI(sensor_msgs::PointCloud2 pc_msg) {
        pcl::PointCloud<pcl::PointXYZINormal> pcl_pc;
        for (int i=0; i<pc_msg.row_step; i+=pc_msg.point_step) {
            pcl::PointXYZINormal point{
                bytes_to_float(&pc_msg.data[i]),
                bytes_to_float(&pc_msg.data[i + 1]),
                bytes_to_float(&pc_msg.data[i + 2]),
                bytes_to_float(&pc_msg.data[i + 3]),
            };
            pcl_pc.points.push_back(point);
        }
        return pcl_pc;
    };
}