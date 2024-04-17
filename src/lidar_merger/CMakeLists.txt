cmake_minimum_required(VERSION 3.0.2)
project(lidar_merger)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
