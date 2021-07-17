cmake_minimum_required(VERSION 3.0.2)
project(traffic_signal_detection)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES traffic_signal_detection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   DEPENDS opencv2
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)