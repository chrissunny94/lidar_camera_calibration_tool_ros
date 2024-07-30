#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert ROS message to PCL point cloud
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*msg, *cloud);

    ROS_INFO("Received point cloud with %zu points", cloud->points.size());

    // Create the PassThrough filter
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    
    // Filter X axis
    pass.setFilterFieldName("x");
    pass.setFilterLimits(1.0, 6.0);
    pass.filter(*cloud);

    // Filter Y axis
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5.0, 5.0);
    pass.filter(*cloud);

    // Filter Z axis
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 2.0);
    pass.filter(*cloud);

    ROS_INFO("Filtered point cloud to %zu points", cloud->points.size());

    // Plane segmentation
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    ROS_INFO("Plane model coefficients: %f %f %f %f",
              coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    // Extract inliers
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    PointCloudT::Ptr cloud_plane(new PointCloudT);
    extract.filter(*cloud_plane);

    ROS_INFO("Extracted plane cloud with %zu points", cloud_plane->points.size());

    // Save filtered cloud to PCD file
    pcl::io::savePCDFileASCII("filtered_cloud.pcd", *cloud_plane);

    // Visualize
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud_plane);
    while (!viewer.wasStopped()) {}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_detection_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/innovusion_pointcloud", 1, pointCloudCallback);

    ros::spin();

    return 0;
}
