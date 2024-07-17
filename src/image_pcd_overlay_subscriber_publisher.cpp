#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/conversions.h>


// ROS and OpenCV includes
void convertPointCloud2ToPCL(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
cv::Mat createPointCloudImageOverlay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const cv::Mat& image, const cv::Mat& camera_matrix);

class ImagePCDSubscriberPublisher {
public:
    ImagePCDSubscriberPublisher(ros::NodeHandle& nh)
        : nh_(nh), slider_value_(50)
    {
        pcd_sub_ = nh_.subscribe("/pandar", 1, &ImagePCDSubscriberPublisher::pcdCallback, this);
        img_sub_ = nh_.subscribe("/entron_camera_node_cam_1_corrected_rgb8", 1, &ImagePCDSubscriberPublisher::imgCallback, this);
        overlay_pub_ = nh_.advertise<sensor_msgs::Image>("front_PCD_overlay", 1);

        // Initialize OpenCV window and slider
        cv::namedWindow("Overlay Window", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("Parameter", "Overlay Window", &slider_value_, 100, onTrackbarSlide, this);
        
        // Create a timer to update the overlay
        timer_ = nh_.createTimer(ros::Duration(0.1), &ImagePCDSubscriberPublisher::timerCallback, this);
    }

    static void onTrackbarSlide(int pos, void* userdata)
    {
        ImagePCDSubscriberPublisher* self = reinterpret_cast<ImagePCDSubscriberPublisher*>(userdata);
        self->slider_value_ = pos;
        self->updateOverlay();
    }

    void updateOverlay()
    {
        if (!cv_image_.empty() && pcl_cloud_)
        {
            cv::Mat camera_matrix; // Set this according to your camera parameters
            cv::Mat overlay = createPointCloudImageOverlay(pcl_cloud_, cv_image_, camera_matrix);
            cv::imshow("Overlay Window", overlay);
            cv::waitKey(1); // Needed to update the OpenCV window
        }
    }

    void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        convertPointCloud2ToPCL(cloud_msg, pcl_cloud);
        pcl_cloud_ = pcl_cloud;
        updateOverlay();
    }

    void imgCallback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        try
        {
            cv_image_ = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        updateOverlay();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pcd_sub_;
    ros::Subscriber img_sub_;
    ros::Publisher overlay_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;
    cv::Mat cv_image_;
    int slider_value_;
    ros::Timer timer_;
};

// Function to convert PointCloud2 to PCL PointCloud
void convertPointCloud2ToPCL(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
{
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);  // Ensure pcl::fromROSMsg is used correctly
}

// Function to create an overlay of the point cloud on the image
cv::Mat createPointCloudImageOverlay(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, const cv::Mat& image, const cv::Mat& camera_matrix)
{
    cv::Mat overlay = image.clone(); // Start with the original image
    // Add logic to project PCL points onto the image

    for (const auto& point : pcl_cloud->points)
    {
        // Example projection logic (customize as needed)
        int x = static_cast<int>(point.x); // Replace with proper projection logic
        int y = static_cast<int>(point.y); // Replace with proper projection logic

        // Draw point on overlay if within bounds
        if (x >= 0 && x < overlay.cols && y >= 0 && y < overlay.rows)
        {
            overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // Green point
        }
    }
    return overlay;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pcd_overlay_subscriber_publisher");
    ros::NodeHandle nh;

    ImagePCDSubscriberPublisher ipo(nh);

    ros::spin();
    return 0;
}
