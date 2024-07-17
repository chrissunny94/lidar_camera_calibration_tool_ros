#ifndef TRANSFORMATIONS_HPP
#define TRANSFORMATIONS_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

cv::Mat computeTransformationMatrix(const cv::Vec3f& rotation, const cv::Vec3f& translation);
pcl::PointXYZ transformPoint(const pcl::PointXYZ& point, const cv::Mat& transform_matrix);

#endif // TRANSFORMATIONS_HPP
