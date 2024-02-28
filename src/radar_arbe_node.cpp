// sys
#include <string>
#include <vector>

// ros
#include "../include/msg/ArbeRadarMsg.h"
#include "../include/msg/ArbeRadarPoint.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// custom
#include "./radar_alg/radar_alg_main.h"

// about pointpillars
#include <cuda_runtime.h>
#include "./pointpillars_singleHead/src/pointpillar/pointpillar.hpp"


ros::Publisher marker_array_pub_;
RadarExp::radarAlg *radarRuner = nullptr;

std::shared_ptr<pointpillar::lidar::Core> pp_core;
cudaStream_t stream;


void GetDeviceInfo(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}

void pub_boxes(const std::vector<Bndbox> &boxes)
{
    visualization_msgs::MarkerArray marker_array;
    tf2::Quaternion myQuaternion;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    uint32_t id = 0;
    for (auto box : boxes)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "mmRadar";
        marker.header.stamp = ros::Time::now();

        marker.id = id;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = box.x;
        marker.pose.position.y = box.y;
        marker.pose.position.z = box.z;

        // std::cout << "box.rt: " << box.rt / M_PI * 180.0 << std::endl;

        myQuaternion.setRPY(0, 0, box.rt);
        marker.pose.orientation = tf2::toMsg(myQuaternion);

        // marker.scale.x = box.l;
        // marker.scale.y = box.w;
        marker.scale.x = box.w;
        marker.scale.y = box.l;
        marker.scale.z = box.h;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.color.a = 0.4;

        marker.lifetime = ros::Duration(1);
        marker_array.markers.push_back(marker);
        id++;
    }

    marker_array_pub_.publish(marker_array);
}

std::shared_ptr<pointpillar::lidar::Core> create_core()
{
    pointpillar::lidar::VoxelizationParameter vp;
    vp.min_range = nvtype::Float3(-39.68f, -10.0f, -3.0);
    vp.max_range = nvtype::Float3(39.68f, 92.4f, 1.0);
    vp.voxel_size = nvtype::Float3(0.32f, 0.32f, 4.0f);
    vp.grid_size = vp.compute_grid_size(vp.max_range, vp.min_range, vp.voxel_size);
    vp.max_voxels = 40000;
    vp.max_points_per_voxel = 32;
    vp.max_points = 300000;
    vp.num_feature = 4;

    pointpillar::lidar::PostProcessParameter pp;
    pp.min_range = vp.min_range;
    pp.max_range = vp.max_range;
    pp.feature_size = nvtype::Int2(vp.grid_size.x / 2, vp.grid_size.y / 2);

    pointpillar::lidar::CoreParameter param;
    param.voxelization = vp;
    param.lidar_model = "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars_singleHead/model/pointpillar.plan"; // 模型绝对路径
    param.lidar_post = pp;
    return pointpillar::lidar::create_core(param);
}

void callbackCloud(const DualRadar::ArbeRadarMsg &arbe_msg)
{
    ROS_INFO_STREAM("\033[1;32m"
                    << "Rec Arbe_msg: "
                    << "\033[0m" << arbe_msg.point_num);

    int num_points = arbe_msg.point_num;
    float *points_data = new float[num_points * 4]; // 4指的是点云特征数
    for(int i=0; i<num_points; i++)
    {
        points_data[4*i + 0] = arbe_msg.points.at(i).x;
        points_data[4*i + 1] = arbe_msg.points.at(i).y;
        points_data[4*i + 2] = arbe_msg.points.at(i).z;
        points_data[4*i + 3] = arbe_msg.points.at(i).rcs;

        // for debug
        if(i < 10)
        {
            std::cout   << points_data[4*i + 0] << ", "
                        << points_data[4*i + 1] << ", " 
                        << points_data[4*i + 2] << ", "
                        << points_data[4*i + 3] << ", " << std::endl;
        }
    }                

    std::cout << "\n<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    auto bboxes = pp_core->forward(points_data, num_points, stream);
    std::cout << "Detections after NMS: " << bboxes.size() << std::endl;
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>" << std::endl;

    delete points_data;
    // // 执行算法部分
    // std::vector<radarPoint> radarPoints;

    // // 转格式
    // for(int i=0; i<arbe_msg.point_num; i++)
    // {
    //     radarPoint new_pc;
    //     new_pc.x = arbe_msg.points.at(i).x;
    //     new_pc.y = arbe_msg.points.at(i).y;
    //     new_pc.z = arbe_msg.points.at(i).z;
    //     new_pc.rcs = arbe_msg.points.at(i).rcs;
    //     new_pc.doppler = arbe_msg.points.at(i).doppler;
    //     radarPoints.push_back(new_pc);
    // }

    // radarRuner->proc(radarPoints);
    // auto boxes = radarRuner->getBoxesOutput();

    // std::vector<Bndbox> boxes;
    // for(int i=0; i<out_labels.size(); i++)
    // {
    //     Bndbox new_Bndbox(out_detections.at(i*7),out_detections.at(i*7+1),out_detections.at(i*7+2),
    //                         out_detections.at(i*7+3),out_detections.at(i*7+4),out_detections.at(i*7+5),
    //                         0.0, 0.0,
    //                         out_detections.at(i*7+6),
    //                         i, out_scores.at(i), out_labels.at(i));

    //     boxes.push_back(new_Bndbox);
    // }

    // pub_boxes(boxes);

    // delete points_array;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arbe");
    ros::NodeHandle nn("~");

    std::string cloud_topic;
    nn.param<std::string>("cloud_topic", cloud_topic, "/arbe_msg");

    // 订阅通道
    ros::Subscriber sub_cloud = nn.subscribe(cloud_topic, 100, callbackCloud);

    // 各种结果的发布通道
    marker_array_pub_ = nn.advertise<visualization_msgs::MarkerArray>("bboxes", 100);

    /**************** 注册pointpillars ******************/
    GetDeviceInfo();
    pp_core = create_core();
    if (pp_core == nullptr)
    {
        printf("Core has been failed.\n");
        return -1;
    }

    pp_core->print();
    pp_core->set_timer(true); // 打印模型各模块运行时间
    cudaStreamCreate(&stream);
    /***************************************************/

    ros::spin();

    return 0;
}
