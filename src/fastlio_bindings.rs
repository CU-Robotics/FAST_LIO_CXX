#[cxx::bridge]
mod ffi {
    // Shared structs (match the *Msg types above)
    struct TimeMsg { sec: i64, nsec: i64 }
    struct HeaderMsg { seq: u32, stamp: TimeMsg, frame_id: String }

    #[namespace = "geometry_msgs"]
    struct Vector3Msg { x: f32, y: f32, z: f32 }
    #[namespace = "geometry_msgs"]
    struct QuaternionMsg { x: f32, y: f32, z: f32, w: f32 }
    #[namespace = "geometry_msgs"]
    struct PoseMsg { position: Vector3Msg, orientation: QuaternionMsg }
    #[namespace = "geometry_msgs"]
    struct PoseStampedMsg { header: HeaderMsg, pose: PoseMsg }
    #[namespace = "geometry_msgs"]
    struct TwistMsg { linear: Vector3Msg, angular: Vector3Msg }

    // Output 6x6 covariance matrices will have length 36. This isn't being enforced
    // programmatically in Rust because CXX does not support fixed-size array bindings.
    #[namespace = "geometry_msgs"]
    struct PoseWithCovarianceMsg { pose: PoseMsg, covariance: Vec<f64> }
    // Output 6x6 covariance matrices will have length 36. This isn't being enforced
    // programmatically in Rust because CXX does not support fixed-size array bindings.
    #[namespace = "geometry_msgs"]
    struct TwistWithCovarianceMsg { twist: TwistMsg, covariance: Vec<f64> }

    #[namespace = "sensor_msgs"]
    struct PointFieldMsg { name: String, offset: u32, datatype: u8, count: u32 }
    #[namespace = "sensor_msgs"]
    struct PointCloud2Msg {
        header: HeaderMsg,
        height: u32,
        width: u32,
        is_bigendian: bool,
        point_step: u32,
        row_step: u32,
        is_dense: bool,
        data: Vec<u8>,
        fields: Vec<PointFieldMsg>,
    }
    #[namespace = "sensor_msgs"]
    struct ImuMsg {
        // Input 3x3 covariance matrices should have length 9. This isn't being enforced
        // programmatically in Rust because CXX does not support fixed-size array bindings.
        header: HeaderMsg,
        orientation: QuaternionMsg,
        orientation_covariance: Vec<f32>, // len 9
        angular_velocity: Vector3Msg,
        angular_velocity_covariance: Vec<f32>, // len 9
        linear_acceleration: Vector3Msg,
        linear_acceleration_covariance: Vec<f32>, // len 9
    }

    #[namespace = "nav_msgs"]
    struct OdometryMsg {
        header: HeaderMsg,
        child_frame_id: String,
        pose: PoseWithCovarianceMsg,
        twist: TwistWithCovarianceMsg,
    }
    #[namespace = "nav_msgs"]
    struct PathMsg { header: HeaderMsg, poses: Vec<PoseStampedMsg> }

    struct LidarOutputMsg {
        has_laser_cloud_full: bool,
        has_laser_cloud_full_body: bool,
        has_laser_cloud_effect: bool,
        has_laser_cloud_map: bool,
        has_odom: bool,
        has_path: bool,
        laser_cloud_full: PointCloud2Msg,
        laser_cloud_full_body: PointCloud2Msg,
        laser_cloud_effect: PointCloud2Msg,
        laser_cloud_map: PointCloud2Msg,
        odom: OdometryMsg,
        path: PathMsg,
    }

    unsafe extern "C++" {
        include!("fastlio_bridge.hxx");

        fn fastlio_init();
        fn fastlio_save_map();
        fn fastlio_run() -> UniquePtr<LidarOutputMsg>;
        fn fastlio_imu_cbk(msg: &ImuMsg);
        fn fastlio_pcl_cbk(msg: &PointCloud2Msg);
    }
}

pub use ffi::*;
