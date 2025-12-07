#ifndef MSG_H
#define MSG_H

#include <memory.h>
#include <time.h>
#include <stdint.h>
#include <vector>

namespace fast_lio {
    struct Pose6D {
        float offset_time;  // the offset time of IMU measurement w.r.t the first lidar point
        float acc[3];       // the preintegrated total acceleration (global frame) at the Lidar origin
        float gyr[3];       // the unbiased angular velocity (body frame) at the Lidar origin
        float vel[3];       // the preintegrated velocity (global frame) at the Lidar origin
        float pos[3];       // the preintegrated position (global frame) at the Lidar origin
        float rot[3];       // the preintegrated rotation (global frame) at the Lidar origin
    };
}

class Time {
  public:
    long sec;
    long nsec;

    Time() : sec(0), nsec(0) {};
    Time(long s, long ns) : sec(s), nsec(ns) {};

    float toSec() const {
        return this->sec + this->nsec / 1000000000;
    };

    long long toNSec() const {
        return this->sec * 1000000000 + this->nsec;
    };

    static Time fromSec(float sec) {
        double integer_sec;
        double decimal_nsec;
        decimal_nsec = modf(sec, &integer_sec);
        return Time(long(integer_sec), long(decimal_nsec * 1000000000));
    }
};

struct Header {
    uint32_t seq;
    Time stamp;
    std::string frame_id;
};

namespace geometry_msgs {
    struct Vector3 {
        float x;
        float y;
        float z;
    };

    struct Quaternion {
        float x;
        float y;
        float z;
        float w;
    };

    struct Pose {
        Vector3 position;
        Quaternion orientation;
    };

    struct PoseWithCovariance {
        Pose pose;
        float covariance[36];
    };

    struct PoseStamped {
        Header header;
        Pose pose;
    };

    struct Twist {
        Vector3 linear;
        Vector3 angular;
    };

    struct TwistWithCovariance {
        Twist twist;
        float covariance[36];
    };
}

namespace sensor_msgs {
    struct Imu {
        Header header;
        geometry_msgs::Quaternion orientation;
        float orientation_covariance[9];
        geometry_msgs::Vector3 angular_velocity;
        float angular_velocity_covariance[9];
        geometry_msgs::Vector3 linear_acceleration;
        float linear_acceleration_covariance[9];

        typedef std::shared_ptr<Imu> Ptr;
        typedef std::shared_ptr<Imu const> ConstPtr;
    };
    typedef std::shared_ptr<Imu const> ImuConstPtr;

    struct PointField {
        std::string name;        // Name of field
        uint32_t offset;    // Offset from start of point struct
        uint8_t  datatype;  // Datatype enumeration, see above
        uint32_t count;     // How many elements in the field
    };

    struct PointCloud2 {
        Header header;
        uint32_t height;
        uint32_t width;
        bool is_bigendian;
        uint32_t point_step;
        uint32_t row_step;
        bool is_dense;
        std::vector<uint8_t> data;
        std::vector<PointField> fields;

        typedef std::shared_ptr<PointCloud2 const> ConstPtr;
    };
    typedef std::shared_ptr<PointCloud2 const> PointCloud2ConstPtr;
}

namespace nav_msgs {
    struct Path {
        Header header;
        std::vector<geometry_msgs::PoseStamped> poses;
    };

    struct Odometry {
        Header header;
        std::string child_frame_id;
        geometry_msgs::PoseWithCovariance pose;
        geometry_msgs::TwistWithCovariance twist;
    };
}

#endif