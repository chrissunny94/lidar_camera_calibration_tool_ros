#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include "lidar_camera_calibration.hpp"

class LidarCameraCalibration : public rclcpp::Node {
public:
    LidarCameraCalibration() : Node("lidar_camera_calibration") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_topic", 10, std::bind(&LidarCameraCalibration::imageCallback, this, std::placeholders::_1));
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_topic", 10, std::bind(&LidarCameraCalibration::lidarCallback, this, std::placeholders::_1));
        overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("overlay_topic", 10);

        // Initialize sliders
        cv::namedWindow("Overlay");
        cv::createTrackbar("x", "Overlay", &x_slider_, 100, onTrackbar, this);
        cv::createTrackbar("y", "Overlay", &y_slider_, 100, onTrackbar, this);
        cv::createTrackbar("z", "Overlay", &z_slider_, 100, onTrackbar, this);
        cv::createTrackbar("roll", "Overlay", &roll_slider_, 360, onTrackbar, this);
        cv::createTrackbar("pitch", "Overlay", &pitch_slider_, 360, onTrackbar, this);
        cv::createTrackbar("yaw", "Overlay", &yaw_slider_, 360, onTrackbar, this);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            current_image_ = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Process and transform point cloud here

        if (!current_image_.empty()) {
            cv::Mat overlay_image = createOverlayImage(cloud, current_image_);
            sensor_msgs::msg::Image::SharedPtr overlay_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay_image).toImageMsg();
            overlay_pub_->publish(*overlay_msg);
            cv::imshow("Overlay", overlay_image);
            cv::waitKey(1);
        }
    }

    static void onTrackbar(int, void* userdata) {
        LidarCameraCalibration* self = static_cast<LidarCameraCalibration*>(userdata);
        // Update the transformation parameters based on slider values
        self->x_ = self->x_slider_ / 100.0;
        self->y_ = self->y_slider_ / 100.0;
        self->z_ = self->z_slider_ / 100.0;
        self->roll_ = self->roll_slider_ * M_PI / 180.0;
        self->pitch_ = self->pitch_slider_ * M_PI / 180.0;
        self->yaw_ = self->yaw_slider_ * M_PI / 180.0;
        // Apply transformations to the point cloud or update visualization
    }

    cv::Mat createOverlayImage(const pcl::PointCloud<pcl::PointXYZ>& cloud, const cv::Mat& image) {
        // Implement point cloud projection and overlay logic using current x_, y_, z_, roll_, pitch_, yaw_
        return image; // Placeholder, replace with actual overlay logic
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
    cv::Mat current_image_;

    // Slider values
    int x_slider_ = 50;
    int y_slider_ = 50;
    int z_slider_ = 50;
    int roll_slider_ = 0;
    int pitch_slider_ = 0;
    int yaw_slider_ = 0;

    // Transformation parameters
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarCameraCalibration>());
    rclcpp::shutdown();
    return 0;
}
