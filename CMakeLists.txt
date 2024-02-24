cmake_minimum_required(VERSION 2.8.7)
project(DualRadar)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  image_transport
  message_generation
)

add_message_files(
   FILES
   ArbeRadarMsg.msg
   ArbeRadarPoint.msg
 )
 
generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package()

set(PCL_INCLUDE_DIRS "usr/include/pcl-1.10/pcl")
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
set(OpenCV_INCLUDE_DIRS "usr/local/include/opencv4/opencv2")
set(Eigen_INCLUDE_DIRS "usr/local/include/eigen3")


include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)

# aux_source_directory函数添加指定文件夹下的cpp文件 
aux_source_directory(. sources)
aux_source_directory(src/common_lib sources) 

#radar_tracking_demo
add_executable(file2bag src/main.cpp src/radar_alg_main.cpp )
target_sources(file2bag PUBLIC ${sources})
target_link_libraries(file2bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(radarExp src/main.cpp src/radar_alg_main.cpp )
target_sources(radarExp PUBLIC ${sources})
target_link_libraries(radarExp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})




