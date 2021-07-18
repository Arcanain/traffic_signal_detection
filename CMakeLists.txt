cmake_minimum_required(VERSION 3.0.2)
project(traffic_signal_detection)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

#find_package(OpenCV REQUIRED)

find_package(OpenCV REQUIRED
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_highgui
  CONFIG
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traffic_signal_detection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  DEPENDS opencv2
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS}
#)

#add_executable(signal_publisher_node src/signal_publisher.cpp include/signal_detector.h)
#add_dependencies(signal_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(signal_publisher_node ${catkin_LIBRARIES})

#add_executable(signal_publisher_node src/test.cpp)
#add_dependencies(signal_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(signal_publisher_node ${catkin_LIBRARIES})


#add_executable(traffic_signal_detection src/test.cpp)
add_executable(signal_publisher src/signal_publisher.cpp)
#add_executable(signal_publisher signal_publisher.cpp)
target_link_libraries(signal_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#install(DIRECTORY include
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

