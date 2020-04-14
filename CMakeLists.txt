cmake_minimum_required(VERSION 2.8.3)
project(hps_camera)

add_compile_options(-std=c++11)

link_directories(${CMAKE_SOURCE_DIR}/common/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  cv_bridge
)

generate_dynamic_reconfigure_options(
        cfg/HPS.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(ros_hps_camera
#  ros/src/ros_hps_camera.cpp
#  ros/src/depth_image_converter.cpp
#  ros/src/point_cloud_converter.cpp
#)
#
#target_link_libraries(
#  ros_hps_camera
#  ${catkin_LIBRARIES}
#  hps3d64179
#)

### new node
find_package(OpenCV 3 REQUIRED)

message("CPU architecture: " ${CMAKE_HOST_SYSTEM_PROCESSOR})
if( ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64" )
    set( HPSLIB "hps3d64" )
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "arm")
    set( HPSLIB "hps3d_arm" )
else()
    set( HPSLIB "hps3d32" )
endif()
message("Using libhps: " ${HPSLIB})


add_executable(hps_node
        ros/src/hps_node.cpp
        ros/src/HPS3DLidar.cpp
        ros/src/depth_image_converter.cpp
        ros/src/point_cloud_converter.cpp
        )

add_dependencies(hps_node ${PROJECT_NAME}_gencfg)

target_link_libraries(
        hps_node
        ${catkin_LIBRARIES}
        ${HPSLIB}
        opencv_core
)