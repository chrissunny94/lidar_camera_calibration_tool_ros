#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "tf/transform_listener.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "geometry_msgs/PointStamped.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

class CameraLidarOverlay {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher overlayed_image_pub;
    ros::Subscriber velodyne_info_sub;
    ros::Subscriber camera_info_sub;
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    pcl::PointCloud<pcl::PointXYZ> velodyne_pc;
    image_geometry::PinholeCameraModel cam_model;
    cv::Mat image;
    ros::Publisher image_fused_pcl_pub;

    // Trackbar variables
    int slider_value_x;
    int slider_value_y;
    int slider_value_z;
    int slider_value_rx;
    int slider_value_ry;
    int slider_value_rz;

public:
    CameraLidarOverlay();
    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg);
    std::vector<geometry_msgs::PointStamped> transform_point_cloud();
    void update_overlay();
    void on_translation_trackbar_slide(int pos, void* userdata);
    void on_rotation_trackbar_slide(int pos, void* userdata);
};

CameraLidarOverlay::CameraLidarOverlay():
    it(nh),
    velodyne_info_sub(nh.subscribe<sensor_msgs::PointCloud2>("/sensors/velodyne_points", 1, &CameraLidarOverlay::lidar_callback, this)),
    image_sub(it.subscribe("/sensors/camera/image_rect_color", 1, &CameraLidarOverlay::image_callback, this)),
    camera_info_sub(nh.subscribe<sensor_msgs::CameraInfo>("/sensors/camera/camera_info", 1, &CameraLidarOverlay::camera_info_callback, this)),
    overlayed_image_pub(it.advertise("/sensors/camera/overlayed_lidar_image", 1)),
    image_fused_pcl_pub(nh.advertise<sensor_msgs::PointCloud2>("/image_fused_pointcloud", 1)),
    slider_value_x(0), slider_value_y(0), slider_value_z(0),
    slider_value_rx(0), slider_value_ry(0), slider_value_rz(0)
{
    cv::namedWindow("Overlay Window", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Translation X", "Overlay Window", &slider_value_x, 100, boost::bind(&CameraLidarOverlay::on_translation_trackbar_slide, this, _1, _2));
    cv::createTrackbar("Translation Y", "Overlay Window", &slider_value_y, 100, boost::bind(&CameraLidarOverlay::on_translation_trackbar_slide, this, _1, _2));
    cv::createTrackbar("Translation Z", "Overlay Window", &slider_value_z, 100, boost::bind(&CameraLidarOverlay::on_translation_trackbar_slide, this, _1, _2));
    cv::createTrackbar("Rotation X", "Overlay Window", &slider_value_rx, 360, boost::bind(&CameraLidarOverlay::on_rotation_trackbar_slide, this, _1, _2));
    cv::createTrackbar("Rotation Y", "Overlay Window", &slider_value_ry, 360, boost::bind(&CameraLidarOverlay::on_rotation_trackbar_slide, this, _1, _2));
    cv::createTrackbar("Rotation Z", "Overlay Window", &slider_value_rz, 360, boost::bind(&CameraLidarOverlay::on_rotation_trackbar_slide, this, _1, _2));
}

void CameraLidarOverlay::lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::fromROSMsg(*msg, this->velodyne_pc);
}

void CameraLidarOverlay::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        this->image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }
    cv::Mat image_copy = this->image.clone();
    pcl::PointCloud<pcl::PointXYZ> transformed_velodyne_pc;
    try {
        tf_listener.waitForTransform("world", "velodyne", ros::Time(0), ros::Duration(10.0));
    } catch (tf::TransformException e) {
        ROS_ERROR("%s", e.what());
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> image_fused_pc;
    pcl::PointXYZRGB image_fused_pt;
    std::vector<geometry_msgs::PointStamped> transformed_pt_vector = transform_point_cloud();
    for (auto transformed_pt : transformed_pt_vector) {
        double x = transformed_pt.point.x, y = transformed_pt.point.y, z = transformed_pt.point.z;
        cv::Point3d xyz(x, y, z);
        cv::Point2d uv = cam_model.project3dToPixel(xyz);
        if (uv.x >= 0 && uv.x < image.cols && uv.y >= 0 && uv.y < image.rows) {
            cv::circle(this->image, uv, 3.0, cv::Scalar(0, 255, 0));
            image_fused_pt.x = x;
            image_fused_pt.y = y;
            image_fused_pt.z = z;
            cv::Vec3b intensity = image_copy.at<cv::Vec3b>(static_cast<int>(uv.y), static_cast<int>(uv.x));
            image_fused_pt.b = intensity.val[0];
            image_fused_pt.g = intensity.val[1];
            image_fused_pt.r = intensity.val[2];
            image_fused_pc.push_back(image_fused_pt);
        }
    }
    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->image).toImageMsg();
    overlayed_image_pub.publish(pub_msg);
    sensor_msgs::PointCloud2 image_fused_pc_msg;
    pcl::toROSMsg(image_fused_pc, image_fused_pc_msg);
    image_fused_pc_msg.header.stamp = ros::Time::now();
    image_fused_pc_msg.header.frame_id = "world";
    image_fused_pcl_pub.publish(image_fused_pc_msg);
    cv::imshow("Overlay Window", this->image);
    cv::waitKey(30);
}

void CameraLidarOverlay::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg) {
    cam_model.fromCameraInfo(msg);
}

std::vector<geometry_msgs::PointStamped> CameraLidarOverlay::transform_point_cloud() {
    std::vector<geometry_msgs::PointStamped> transformed_pt_vector;
    geometry_msgs::PointStamped pt, transformed_pt;
    for (size_t i = 0; i < velodyne_pc.points.size(); i++) {
        pt.header.frame_id = "velodyne";
        pt.header.stamp = ros::Time();
        pt.point.x = velodyne_pc.points[i].x;
        pt.point.y = velodyne_pc.points[i].y;
        pt.point.z = velodyne_pc.points[i].z;
        tf_listener.transformPoint("world", pt, transformed_pt);
        transformed_pt_vector.push_back(transformed_pt);
    }
    return transformed_pt_vector;
}

void CameraLidarOverlay::on_translation_trackbar_slide(int pos, void* userdata) {
    slider_value_x = cv::getTrackbarPos("Translation X", "Overlay Window");
    slider_value_y = cv::getTrackbarPos("Translation Y", "Overlay Window");
    slider_value_z = cv::getTrackbarPos("Translation Z", "Overlay Window");

    Eigen::Matrix4f T_lidar_camera = Eigen::Matrix4f::Identity();
    T_lidar_camera(0, 3) = static_cast<float>(slider_value_x) / 100.0f;
    T_lidar_camera(1, 3) = static_cast<float>(slider_value_y) / 100.0f;
    T_lidar_camera(2, 3) = static_cast<float>(slider_value_z) / 100.0f;

    Eigen::Matrix4f T_lidar_camera_inv = T_lidar_camera.inverse();
    update_overlay();
}

void CameraLidarOverlay::on_rotation_trackbar_slide(int pos, void* userdata) {
    slider_value_rx = cv::getTrackbarPos("Rotation X", "Overlay Window");
    slider_value_ry = cv::getTrackbarPos("Rotation Y", "Overlay Window");
    slider_value_rz = cv::getTrackbarPos("Rotation Z", "Overlay Window");

    float rx = static_cast<float>(slider_value_rx) * CV_PI / 180.0f;
    float ry = static_cast<float>(slider_value_ry) * CV_PI / 180.0f;
    float rz = static_cast<float>(slider_value_rz) * CV_PI / 180.0f;

    Eigen::AngleAxisf rx_angle(rx, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ry_angle(ry, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rz_angle(rz, Eigen::Vector3f::UnitZ());

    Eigen::Matrix3f rotation_matrix = (rz_angle * ry_angle * rx_angle).matrix();

    Eigen::Matrix4f T_lidar_camera = Eigen::Matrix4f::Identity();
    T_lidar_camera.block<3, 3>(0, 0) = rotation_matrix;

    Eigen::Matrix4f T_lidar_camera_inv = T_lidar_camera.inverse();
    update_overlay();
}

void CameraLidarOverlay::update_overlay() {
    if (!image.empty() && !velodyne_pc.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>(velodyne_pc));
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f transform(T_lidar_camera_inv);
        pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);
        overlayed_image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_lidar_overlay");
    CameraLidarOverlay camera_lidar_overlay;
    ros::spin();
    return 0;
}
