#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace std;

// Camera parameters
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 500.0, 0, 320.0, 
                                                      0, 500.0, 240.0, 
                                                      0, 0, 1);
cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
Eigen::Matrix4f T_lidar_camera = Eigen::Matrix4f::Identity();
Eigen::Matrix4f T_lidar_camera_inv = Eigen::Matrix4f::Identity();
bool calibration_done = false;

// ROS objects
ros::Subscriber pcd_sub_;
ros::Subscriber img_sub_;
ros::Publisher overlay_pub_;
ros::Publisher original_img_pub_;
ros::NodeHandle* nh_;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;
cv::Mat cv_image_;
cv::Mat overlay_;

// Function declarations
void convertPointCloud2ToPCL(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
cv::Mat createPointCloudImageOverlay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const cv::Mat& image, const cv::Mat& camera_matrix);
void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void imgCallback(const sensor_msgs::ImageConstPtr& img_msg);
void calibrateCameraLidar();
void findCheckerboardInPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, std::vector<cv::Point3f>& objpoints, std::vector<cv::Point2f>& imgpoints);
void computeExtrinsics(const std::vector<cv::Point3f>& objpoints, const std::vector<cv::Point2f>& imgpoints);
void updateOverlay();
void showOriginalImage(const cv::Mat& image);
void detectPlaneRansac(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud, Eigen::VectorXf& plane_coefficients);

void convertPointCloud2ToPCL(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
}

void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    convertPointCloud2ToPCL(cloud_msg, pcl_cloud);
    pcl_cloud_ = pcl_cloud;

    // Perform calibration if not done yet
    if (!calibration_done && !cv_image_.empty()) {
        calibrateCameraLidar();
    }
}

void imgCallback(const sensor_msgs::ImageConstPtr& img_msg) {
    try {
        cv::Mat full_image = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::Mat resized_image;
        cv::resize(full_image, resized_image, cv::Size(full_image.cols / 2, full_image.rows / 2));
        cv_image_ = resized_image;

        // Show original image in a separate window
        showOriginalImage(full_image);

        // Optionally, find checkerboard corners here
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void showOriginalImage(const cv::Mat& image) {
    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(image.cols / 2, image.rows / 2));
    cv::imshow("Original Image", resized_image);
    cv::waitKey(1);
}

void detectPlaneRansac(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud, Eigen::VectorXf& plane_coefficients) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(pcl_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);  // Distance threshold for plane fitting

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pcl_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    plane_coefficients = Eigen::VectorXf(4);
    plane_coefficients(0) = coefficients->values[0];
    plane_coefficients(1) = coefficients->values[1];
    plane_coefficients(2) = coefficients->values[2];
    plane_coefficients(3) = coefficients->values[3];
}

void findCheckerboardInPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, std::vector<cv::Point3f>& objpoints, std::vector<cv::Point2f>& imgpoints) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::VectorXf plane_coefficients;

    detectPlaneRansac(pcl_cloud, plane_cloud, plane_coefficients);

    // Map the detected plane points to the checkerboard points
    // Placeholder: This should involve mapping detected points to checkerboard points and then collecting them into objpoints and imgpoints.
}

void computeExtrinsics(const std::vector<cv::Point3f>& objpoints, const std::vector<cv::Point2f>& imgpoints) {
    cv::Mat rvec, tvec;
    std::cout << "Number of object points: " << objpoints.size() << std::endl;
    std::cout << "Number of image points: " << imgpoints.size() << std::endl;
    if (objpoints.size() > 3 && imgpoints.size() > 3) {
        cv::solvePnP(objpoints, imgpoints, camera_matrix, dist_coeffs, rvec, tvec);

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        Eigen::Matrix4f extrinsics = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f rotation_eigen;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotation_eigen(i, j) = rotation_matrix.at<double>(i, j);
            }
        }

        extrinsics.block<3, 3>(0, 0) = rotation_eigen.transpose();
        Eigen::Vector3f translation_eigen;
        for (int i = 0; i < 3; ++i) {
            translation_eigen(i) = tvec.at<double>(i, 0);
        }
        extrinsics.block<3, 1>(0, 3) = -rotation_eigen.transpose() * translation_eigen;

        T_lidar_camera = extrinsics;
        T_lidar_camera_inv = T_lidar_camera.inverse();

        calibration_done = true;
    }
}

void calibrateCameraLidar() {
    std::vector<cv::Point3f> objpoints;
    std::vector<cv::Point2f> imgpoints;
    findCheckerboardInPointCloud(pcl_cloud_, objpoints, imgpoints);
    computeExtrinsics(objpoints, imgpoints);
}

cv::Mat createPointCloudImageOverlay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const cv::Mat& image, const cv::Mat& camera_matrix) {
    cv::Mat overlay = image.clone();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform(T_lidar_camera);
    pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);

    for (const auto& point : transformed_cloud->points) {
        cv::Mat pt(4, 1, CV_64F);
        pt.at<double>(0, 0) = point.x;
        pt.at<double>(1, 0) = point.y;
        pt.at<double>(2, 0) = point.z;
        pt.at<double>(3, 0) = 1.0;

        cv::Mat img_pt = camera_matrix * pt;
        int x = static_cast<int>(img_pt.at<double>(0, 0) / img_pt.at<double>(2, 0));
        int y = static_cast<int>(img_pt.at<double>(1, 0) / img_pt.at<double>(2, 0));

        if (x >= 0 && x < overlay.cols && y >= 0 && y < overlay.rows) {
            overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);  // Green color for LiDAR points
        }
    }
    return overlay;
}

void updateOverlay() {
    if (!cv_image_.empty() && pcl_cloud_ && calibration_done) {
        overlay_ = createPointCloudImageOverlay(pcl_cloud_, cv_image_, camera_matrix);
        cv::imshow("Overlay Window", overlay_);
        cv::waitKey(1);

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr overlay_msg = cv_bridge::CvImage(header, "bgr8", overlay_).toImageMsg();
        overlay_pub_.publish(overlay_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_pcd_overlay_subscriber_publisher");
    ros::NodeHandle nh;

    nh_ = &nh;

    // ROS subscribers and publishers
    pcd_sub_ = nh.subscribe("/innovusion_pointcloud", 1, pcdCallback);
    img_sub_ = nh.subscribe("/entron_camera_node_cam_1_corrected_rgb8", 1, imgCallback);
    overlay_pub_ = nh.advertise<sensor_msgs::Image>("/camera/overlay", 1);
    original_img_pub_ = nh.advertise<sensor_msgs::Image>("/camera/original_image", 1);

    // Spin ROS
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        updateOverlay();
        rate.sleep();
    }

    return 0;
}
