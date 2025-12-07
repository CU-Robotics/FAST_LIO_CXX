mod fastlio_bindings;

pub fn main() {
    fastlio_bindings::fastlio_init();
    let dt = std::time::Duration::from_micros(200); // 5000 Hz
    let mut status = true;
    let mut count = 0;
    while status && count < 100 {
        // Do this when IMU has data
        // imu_cbk(imu_data);

        // Do this when LiDar has PointCloud data
        // standard_pcl_cbk(lidar_data);
        println!("Iteration {}", count);
        let data = fastlio_bindings::fastlio_run();
        std::thread::sleep(dt);
        count += 1;
    }
    println!("Bindings test done. Exiting after {} iterations", count);
    fastlio_bindings::fastlio_save_map();
}
