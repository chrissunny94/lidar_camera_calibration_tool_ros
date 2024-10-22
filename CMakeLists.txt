cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_calibration_tool)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp pcl_conversions pcl_ros sensor_msgs image_transport cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_pcd_overlay_subscriber_publisher
  src/image_pcd_overlay_subscriber_publisher.cpp

)

add_executable(camera_intrinsics
  src/camera_intrinsics.cpp
)
add_executable(camera_lidar_calibration
  src/camera_lidar_calibration.cpp
)

target_link_libraries(image_pcd_overlay_subscriber_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(camera_lidar_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(camera_intrinsics
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(plane_detection
  src/plane_detection.cpp
)

target_link_libraries(plane_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
