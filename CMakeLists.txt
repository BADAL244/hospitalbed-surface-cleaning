cmake_minimum_required(VERSION 3.0.2)
project(objectrecognition)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  ar_track_alvar_msgs
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/Tuning.cfg
)


find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imagenode src/ros_control.cpp 
                        src/ros_control_node.cpp
)
add_dependencies(imagenode objectrecognition_gencfg)

target_link_libraries(imagenode 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
)

add_executable(verticle_node src/ros_verticle.cpp 
                        src/ros_control_node.cpp
)
add_dependencies(verticle_node objectrecognition_gencfg)

target_link_libraries(verticle_node 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
)

add_executable(horizontal_crop src/ros_crop.cpp 
                        src/ros_crop_node.cpp
)
add_dependencies(horizontal_crop objectrecognition_gencfg)

target_link_libraries(horizontal_crop 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
)