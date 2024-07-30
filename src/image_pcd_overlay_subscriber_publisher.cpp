#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>  // For pcl::transformPointCloud
#include <Eigen/Dense>

// Camera parameters (from JSON)
cv::Mat camera_matrix;
Eigen::Matrix4f T_lidar_camera = Eigen::Matrix4f::Identity();
Eigen::Matrix4f T_lidar_camera_inv = Eigen::Matrix4f::Identity();
bool sim_data = false;  // Change to true for simulation data, false for real camera data

// ROS objects
ros::Subscriber pcd_sub_;
ros::Subscriber img_sub_;
ros::Publisher overlay_pub_;
ros::NodeHandle* nh_;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;
cv::Mat cv_image_;
cv::Mat overlay_;

// Function declarations
void convertPointCloud2ToPCL(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
cv::Mat createPointCloudImageOverlay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const cv::Mat& image, const cv::Mat& camera_matrix);
void updateOverlay();
void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void imgCallback(const sensor_msgs::ImageConstPtr& img_msg);

// Function implementations
void convertPointCloud2ToPCL(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
}

cv::Mat createPointCloudImageOverlay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const cv::Mat& image, const cv::Mat& camera_matrix) {
    cv::Mat overlay = image.clone();

    // Apply transformation (T_lidar_camera) to each point in pcl_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform(T_lidar_camera);
    pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);

    // Project points onto the image plane
    for (const auto& point : transformed_cloud->points) {
        // Convert point to homogeneous coordinates
        cv::Mat pt(4, 1, CV_64F);
        pt.at<double>(0, 0) = point.x;
        pt.at<double>(1, 0) = point.y;
        pt.at<double>(2, 0) = point.z;
        pt.at<double>(3, 0) = 1.0;

        // Project the point using the camera matrix
        cv::Mat img_pt = camera_matrix * pt;
        int x = static_cast<int>(img_pt.at<double>(0, 0) / img_pt.at<double>(2, 0));
        int y = static_cast<int>(img_pt.at<double>(1, 0) / img_pt.at<double>(2, 0));

        // Draw LiDAR points on overlay image if within image bounds
        if (x >= 0 && x < overlay.cols && y >= 0 && y < overlay.rows) {
            overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);  // Green color for LiDAR points
        }
    }
    return overlay;
}

void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    convertPointCloud2ToPCL(cloud_msg, pcl_cloud);
    pcl_cloud_ = pcl_cloud;
}

void imgCallback(const sensor_msgs::ImageConstPtr& img_msg) {
    try {
        cv_image_ = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

        // Resize the image to half
        cv::Mat resized_image;
        cv::resize(cv_image_, resized_image, cv::Size(cv_image_.cols / 2, cv_image_.rows / 2));

        // Adjust the camera matrix parameters for the resized image
        cv::Mat adjusted_camera_matrix = camera_matrix.clone();
        adjusted_camera_matrix.at<double>(0, 0) /= 2.0;  // fx
        adjusted_camera_matrix.at<double>(1, 1) /= 2.0;  // fy
        adjusted_camera_matrix.at<double>(0, 2) /= 2.0;  // cx
        adjusted_camera_matrix.at<double>(1, 2) /= 2.0;  // cy

        cv_image_ = resized_image;
        camera_matrix = adjusted_camera_matrix;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void updateOverlay() {
    if (!cv_image_.empty() && pcl_cloud_) {
        overlay_ = createPointCloudImageOverlay(pcl_cloud_, cv_image_, camera_matrix);
        cv::imshow("Overlay Window", overlay_);
        cv::waitKey(1);

        // Publish overlay image
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr overlay_msg = cv_bridge::CvImage(header, "bgr8", overlay_).toImageMsg();
        overlay_pub_.publish(overlay_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_pcd_overlay_subscriber_publisher");
    ros::NodeHandle nh;

    // Camera matrix initialization with provided parameters
    double fx = 2286.307473485605;
    double fy = 1898.872192961642;
    double cx = 897.2227375261391;  // Principal point x
    double cy = 565.8299365695649;  // Principal point y
    camera_matrix = (cv::Mat_<double>(3, 4) <<
        fx, 0, cx,0,
        0, fy, cy,0,
        0, 0, 1,0);

    // Initialize T_lidar_camera from the provided transformation matrix (T_lidar_camera_4x4_row_major)
    T_lidar_camera << -0.02005,  0.52933,  -0.84818,  -0.05184,
                      -0.18328, -0.83592,  -0.51735,  -0.07540,
                      -0.98286,  0.14508,   0.11377,   0.23186,
                       0.00000,  0.00000,   0.00000,   1.00000;

    // Calculate T_lidar_camera_inv (inverse of T_lidar_camera)
    T_lidar_camera_inv = T_lidar_camera.inverse();

    // ROS subscribers and publisher initialization
    pcd_sub_ = nh.subscribe("/innovusion_pointcloud", 1, pcdCallback);
    img_sub_ = nh.subscribe("/entron_camera_node_cam_1_corrected_rgb8", 1, imgCallback);
    overlay_pub_ = nh.advertise<sensor_msgs::Image>("front_PCD_overlay", 1);

    // Create window for visualization
    cv::namedWindow("Overlay Window", cv::WINDOW_AUTOSIZE);

    // Commented out slider functionality
    /*
    // Create trackbars for translation and rotation
    cv::createTrackbar("Translation X", "Overlay Window", &slider_value_x, 100, onTranslationTrackbarSlide);
    cv::createTrackbar("Translation Y", "Overlay Window", &slider_value_y, 100, onTranslationTrackbarSlide);
    cv::createTrackbar("Translation Z", "Overlay Window", &slider_value_z, 100, onTranslationTrackbarSlide);
    cv::createTrackbar("Rotation X", "Overlay Window", &slider_value_rx, 360, onRotationTrackbarSlide);
    cv::createTrackbar("Rotation Y", "Overlay Window", &slider_value_ry, 360, onRotationTrackbarSlide);
    cv::createTrackbar("Rotation Z", "Overlay Window", &slider_value_rz, 360, onRotationTrackbarSlide);
    */

    // Main loop to handle ROS events
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        updateOverlay();
        rate.sleep();
    }

    return 0;
}
