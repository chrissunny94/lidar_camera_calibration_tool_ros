#include "lidar_camera_calibration.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCameraCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
