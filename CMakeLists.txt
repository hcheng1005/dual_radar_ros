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
# set(OpenCV_INCLUDE_DIRS "usr/local/include/opencv4/opencv2")
set(Eigen_INCLUDE_DIRS "usr/local/include/eigen3")

find_package(yaml-cpp REQUIRED)

find_package(CUDA REQUIRED QUIET)
include_directories(${CUDA_INCLUDE_DIRS})
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++17 -Wno-deprecated-gpu-targets)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        # ${OpenCV_INCLUDE_DIRS}
        include
)

# aux_source_directory函数添加指定文件夹下的cpp文件 
aux_source_directory(. sources)
aux_source_directory(src/common_lib sources) 
aux_source_directory(src/*.cpp sources) 
aux_source_directory(src/radar_alg/*.cpp sources) 

# 
add_executable(file2bag src/file2bag.cpp )
target_sources(file2bag PUBLIC ${sources})
target_link_libraries(file2bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



#***********************************************************************************************
# radar 
# 添加*.cu文件
cuda_add_library(gpu_point_pillars_lib
                src/pointpillars/postprocess.cu
                src/pointpillars/scatter.cu
                src/pointpillars/preprocess.cu
                src/pointpillars/nms.cu
)

# 编译gpu相关文件库
target_link_libraries(gpu_point_pillars_lib
                    ${CUDA_LIBRARIES} 
                    ${CUDA_CUBLAS_LIBRARIES}
                    ${CUDA_curand_LIBRARY}
                    ${CUDNN_LIBRARY}
                    nvonnxparser
                    nvinfer
)

add_executable(radar_node src/radar_arbe_node.cpp 
                          src/radar_alg/radar_alg_main.cpp 
                          src/radar_alg/ego_velocity_estimator.cpp 
                          src/pointpillars/pointpillars.cc)

target_sources(radar_node PUBLIC ${sources})
target_link_libraries(radar_node ${catkin_LIBRARIES} yaml-cpp gpu_point_pillars_lib)

#***********************************************************************************************

# lidar
add_executable(lidar_node src/lidar_node.cpp src/pointpillars/pointpillars.cc )
target_sources(lidar_node PUBLIC ${sources})
target_link_libraries(lidar_node ${catkin_LIBRARIES}
                                  yaml-cpp
                                  gpu_point_pillars_lib)

#***********************************************************************************************

# camera
add_executable(camera_node src/camera_node.cpp )
target_sources(camera_node PUBLIC ${sources})
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )




