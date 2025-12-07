#include "fastlio_bridge.hxx"
#include "laserMapping.h"
#include <algorithm>
#include <cstring>

namespace {
// Time helpers
TimeMsg to_ffi_time(const Time &t) { return TimeMsg{t.sec, t.nsec}; }
Time to_time(const TimeMsg &m) { return Time(m.sec, m.nsec); }

Header to_cpp_header(const HeaderMsg &m) { return Header{m.seq, to_time(m.stamp), std::string(m.frame_id)}; }
HeaderMsg to_ffi_header(const Header &m) { return HeaderMsg{m.seq, to_ffi_time(m.stamp), m.frame_id}; }

geometry_msgs::Vector3 to_cpp_vec3(const geometry_msgs::Vector3Msg &m) { return {m.x, m.y, m.z}; }
geometry_msgs::Vector3Msg to_ffi_vec3(const geometry_msgs::Vector3 &m) { return {m.x, m.y, m.z}; }

geometry_msgs::Quaternion to_cpp_quat(const geometry_msgs::QuaternionMsg &m) { return {m.x, m.y, m.z, m.w}; }
geometry_msgs::QuaternionMsg to_ffi_quat(const geometry_msgs::Quaternion &m) { return {m.x, m.y, m.z, m.w}; }

// geometry_msgs::Pose to_cpp_pose(const geometry_msgs::PoseMsg &m) { return {to_cpp_vec3(m.position), to_cpp_quat(m.orientation)}; }
geometry_msgs::PoseMsg to_ffi_pose(const geometry_msgs::Pose &m) { return {to_ffi_vec3(m.position), to_ffi_quat(m.orientation)}; }

// geometry_msgs::PoseStamped to_cpp_pose_stamped(const geometry_msgs::PoseStampedMsg &m) {
//     return {to_cpp_header(m.header), to_cpp_pose(m.pose)};
// }
geometry_msgs::PoseStampedMsg to_ffi_pose_stamped(const geometry_msgs::PoseStamped &m) {
    return {to_ffi_header(m.header), to_ffi_pose(m.pose)};
}

// geometry_msgs::PoseWithCovariance to_cpp_pose_cov(const geometry_msgs::PoseWithCovarianceMsg &m) {
//     geometry_msgs::PoseWithCovariance out;
//     out.pose = to_cpp_pose(m.pose);
//     std::fill_n(out.covariance, 36, 0.0);
//     auto copy_size = std::min<size_t>(36, m.covariance.size());
//     for (size_t i = 0; i < copy_size; ++i) out.covariance[i] = m.covariance[i];
//     return out;
// }
geometry_msgs::PoseWithCovarianceMsg to_ffi_pose_cov(const geometry_msgs::PoseWithCovariance &m) {
    geometry_msgs::PoseWithCovarianceMsg out;
    out.pose = to_ffi_pose(m.pose);
    for (auto cov: m.covariance) { out.covariance.push_back(cov); };
    return out;
}

// geometry_msgs::Twist to_cpp_twist(const geometry_msgs::TwistMsg &m) {
//     return {to_cpp_vec3(m.linear), to_cpp_vec3(m.angular)};
// }
geometry_msgs::TwistMsg to_ffi_twist(const geometry_msgs::Twist &m) {
    return {to_ffi_vec3(m.linear), to_ffi_vec3(m.angular)};
}

// geometry_msgs::TwistWithCovariance to_cpp_twist_cov(const geometry_msgs::TwistWithCovarianceMsg &m) {
//     geometry_msgs::TwistWithCovariance out;
//     out.twist = to_cpp_twist(m.twist);
//     std::fill_n(out.covariance, 36, 0.0);
//     auto copy_size = std::min<size_t>(36, m.covariance.size());
//     for (size_t i = 0; i < copy_size; ++i) out.covariance[i] = m.covariance[i];
//     return out;
// }
geometry_msgs::TwistWithCovarianceMsg to_ffi_twist_cov(const geometry_msgs::TwistWithCovariance &m) {
    geometry_msgs::TwistWithCovarianceMsg out;
    out.twist = to_ffi_twist(m.twist);
    for (auto cov: m.covariance) { out.covariance.push_back(cov); };
    return out;
}

sensor_msgs::Imu to_cpp_imu(const sensor_msgs::ImuMsg &m) {
    sensor_msgs::Imu out;
    out.header = to_cpp_header(m.header);
    out.orientation = to_cpp_quat(m.orientation);
    std::fill_n(out.orientation_covariance, 9, 0.0f);
    auto oc_size = std::min<size_t>(9, m.orientation_covariance.size());
    for (size_t i = 0; i < oc_size; ++i) out.orientation_covariance[i] = m.orientation_covariance[i];

    out.angular_velocity = to_cpp_vec3(m.angular_velocity);
    std::fill_n(out.angular_velocity_covariance, 9, 0.0f);
    auto avc_size = std::min<size_t>(9, m.angular_velocity_covariance.size());
    for (size_t i = 0; i < avc_size; ++i) out.angular_velocity_covariance[i] = m.angular_velocity_covariance[i];

    out.linear_acceleration = to_cpp_vec3(m.linear_acceleration);
    std::fill_n(out.linear_acceleration_covariance, 9, 0.0f);
    auto lac_size = std::min<size_t>(9, m.linear_acceleration_covariance.size());
    for (size_t i = 0; i < lac_size; ++i) out.linear_acceleration_covariance[i] = m.linear_acceleration_covariance[i];
    return out;
}

sensor_msgs::PointCloud2 to_cpp_pcl(const sensor_msgs::PointCloud2Msg &m) {
    sensor_msgs::PointCloud2 out;
    out.header = to_cpp_header(m.header);
    out.height = m.height;
    out.width = m.width;
    out.is_bigendian = m.is_bigendian;
    out.point_step = m.point_step;
    out.row_step = m.row_step;
    out.is_dense = m.is_dense;
    for (auto data: m.data) { out.data.push_back(data); };
    out.fields.clear();
    out.fields.reserve(m.fields.size());
    for (const auto &f : m.fields) {
        sensor_msgs::PointField pf;
        pf.name = std::string(f.name);
        pf.offset = f.offset;
        pf.datatype = f.datatype;
        pf.count = f.count;
        out.fields.push_back(std::move(pf));
    }
    return out;
}

// sensor_msgs::ImuMsg to_ffi_imu(const sensor_msgs::Imu &m) {
//     sensor_msgs::ImuMsg out;
//     out.header = to_ffi_header(m.header);
//     out.orientation = to_ffi_quat(m.orientation);
//     for (auto ori_cov: m.orientation_covariance) { out.orientation_covariance.push_back(ori_cov); };
//     out.angular_velocity = to_ffi_vec3(m.angular_velocity);
//     for (auto ori_cov: m.angular_velocity_covariance) { out.angular_velocity_covariance.push_back(ori_cov); };
//     out.linear_acceleration = to_ffi_vec3(m.linear_acceleration);
//     for (auto ori_cov: m.linear_acceleration_covariance) { out.linear_acceleration_covariance.push_back(ori_cov); };
//     return out;
// }

sensor_msgs::PointCloud2Msg to_ffi_pcl(const sensor_msgs::PointCloud2 &m) {
    sensor_msgs::PointCloud2Msg out;
    out.header = to_ffi_header(m.header);
    out.height = m.height;
    out.width = m.width;
    out.is_bigendian = m.is_bigendian;
    out.point_step = m.point_step;
    out.row_step = m.row_step;
    out.is_dense = m.is_dense;
    for(auto data : m.data) { out.data.push_back(data); }
    out.fields.clear();
    out.fields.reserve(m.fields.size());
    for (const auto &f : m.fields) {
        sensor_msgs::PointFieldMsg pf;
        pf.name = f.name;
        pf.offset = f.offset;
        pf.datatype = f.datatype;
        pf.count = f.count;
        out.fields.push_back(std::move(pf));
    }
    return out;
}

nav_msgs::OdometryMsg to_ffi_odom(const nav_msgs::Odometry &m) {
    nav_msgs::OdometryMsg out;
    out.header = to_ffi_header(m.header);
    out.child_frame_id = m.child_frame_id;
    out.pose = to_ffi_pose_cov(m.pose);
    out.twist = to_ffi_twist_cov(m.twist);
    return out;
}

nav_msgs::PathMsg to_ffi_path(const nav_msgs::Path &m) {
    nav_msgs::PathMsg out;
    out.header = to_ffi_header(m.header);
    out.poses.clear();
    out.poses.reserve(m.poses.size());
    for (const auto &p : m.poses) {
        out.poses.push_back(to_ffi_pose_stamped(p));
    }
    return out;
}
} // namespace

void fastlio_init() { init(); }
void fastlio_save_map() { save_map(); }

std::unique_ptr<LidarOutputMsg> fastlio_run() {
    auto res = run();
    if (!res) return nullptr;
    auto out = std::make_unique<LidarOutputMsg>();
    out->has_laser_cloud_full = static_cast<bool>(res->pubLaserCloudFull);
    if (res->pubLaserCloudFull) out->laser_cloud_full = to_ffi_pcl(*res->pubLaserCloudFull);
    out->has_laser_cloud_full_body = static_cast<bool>(res->pubLaserCloudFull_body);
    if (res->pubLaserCloudFull_body) out->laser_cloud_full_body = to_ffi_pcl(*res->pubLaserCloudFull_body);
    out->has_laser_cloud_effect = static_cast<bool>(res->pubLaserCloudEffect);
    if (res->pubLaserCloudEffect) out->laser_cloud_effect = to_ffi_pcl(*res->pubLaserCloudEffect);
    out->has_laser_cloud_map = static_cast<bool>(res->pubLaserCloudMap);
    if (res->pubLaserCloudMap) out->laser_cloud_map = to_ffi_pcl(*res->pubLaserCloudMap);
    out->has_odom = static_cast<bool>(res->pubOdomAftMapped);
    if (res->pubOdomAftMapped) out->odom = to_ffi_odom(*res->pubOdomAftMapped);
    out->has_path = static_cast<bool>(res->pubPath);
    if (res->pubPath) out->path = to_ffi_path(*res->pubPath);
    return out;
}

void fastlio_imu_cbk(const sensor_msgs::ImuMsg &msg) {
    imu_cbk(std::make_shared<sensor_msgs::Imu>(to_cpp_imu(msg)));
}
void fastlio_pcl_cbk(const sensor_msgs::PointCloud2Msg &msg) {
    standard_pcl_cbk(std::make_shared<sensor_msgs::PointCloud2>(to_cpp_pcl(msg)));
}
