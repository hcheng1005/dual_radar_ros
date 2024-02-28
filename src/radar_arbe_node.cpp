// sys
#include <string>
#include <vector>

// ros
#include "../include/msg/ArbeRadarMsg.h"
#include "../include/msg/ArbeRadarPoint.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// custom
#include "./radar_alg/radar_alg_main.h"

// about pointpillars
#include "./pointpillars_singleHead/src/pointpillar/pointpillar.hpp"
#include <cuda_runtime.h>

ros::Publisher marker_array_pub_;
ros::Publisher pub_bbox;
RadarExp::radarAlg *radarRuner = nullptr;

std::shared_ptr<pointpillar::lidar::Core> pp_core;
cudaStream_t stream;

/**
 * @name: 
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void GetDeviceInfo(void)
{
    cudaDeviceProp prop;

    int count = 0;
    cudaGetDeviceCount(&count);
    printf("\nGPU has cuda devices: %d\n", count);
    for (int i = 0; i < count; ++i)
    {
        cudaGetDeviceProperties(&prop, i);
        printf("----device id: %d info----\n", i);
        printf("  GPU : %s \n", prop.name);
        printf("  Capbility: %d.%d\n", prop.major, prop.minor);
        printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
        printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
        printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
        printf("  warp size: %d\n", prop.warpSize);
        printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
        printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
        printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
    }
    printf("\n");
}

/**
 * @name: 
 * @description: Briefly describe the function of your function
 * @param {vector<Bndbox>} &boxes
 * @return {*}
 */
void pub_boxes(const std::vector<pointpillar::lidar::BoundingBox> &bboxes)
{
    jsk_recognition_msgs::BoundingBoxArray jsk_boxes;
    // box_dim： x，y，z，dx，dy，dz，yaw
    for (int i = 0; i < bboxes.size(); i++)
    {
        jsk_recognition_msgs::BoundingBox jsk_box;
        jsk_box.header.frame_id = "map";
        jsk_box.pose.position.x = bboxes.at(i).x;
        jsk_box.pose.position.y = bboxes.at(i).y;
        jsk_box.pose.position.z = bboxes.at(i).z;
        jsk_box.dimensions.x = bboxes.at(i).l;
        jsk_box.dimensions.y = bboxes.at(i).w;
        jsk_box.dimensions.z = bboxes.at(i).h;

        // yaw
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(bboxes.at(i).rt, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond quaternion;
        quaternion = yawAngle * pitchAngle * rollAngle;
        jsk_box.pose.orientation.w = quaternion.w();
        jsk_box.pose.orientation.x = quaternion.x();
        jsk_box.pose.orientation.y = quaternion.y();
        jsk_box.pose.orientation.z = quaternion.z();
   
        jsk_box.label = bboxes.at(i).id;
        jsk_box.value = bboxes.at(i).score;

        jsk_boxes.boxes.emplace_back(jsk_box);
    }
    jsk_boxes.header.frame_id = "map";
    pub_bbox.publish(jsk_boxes);
}

/**
 * @name: create_core
 * @description: 构造pointpillars推理引擎
 * @return {*}
 */
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

/**
 * @name: callbackCloud
 * @description: 消息回调函数，处理接收到的点云数据
 * @param {ArbeRadarMsg} &arbe_msg
 * @return {*}
 */
void callbackCloud(const DualRadar::ArbeRadarMsg &arbe_msg)
{
    ROS_INFO_STREAM("\033[1;32m"
                    << "Rec Arbe_msg: "
                    << "\033[0m" << arbe_msg.point_num);

    int num_points = arbe_msg.point_num;
    float *points_data = new float[num_points * 4]; // 4指的是点云特征数
    for (int i = 0; i < num_points; i++)
    {
        points_data[4 * i + 0] = arbe_msg.points.at(i).x;
        points_data[4 * i + 1] = arbe_msg.points.at(i).y;
        points_data[4 * i + 2] = arbe_msg.points.at(i).z;
        points_data[4 * i + 3] = arbe_msg.points.at(i).rcs;
    }

    std::cout << "\n<<<<<<<<<<<<<<<<<<<<<<" << std::endl;

    // 模型推理
    auto bboxes = pp_core->forward(points_data, num_points, stream);

    // 发布检测结果
    pub_boxes(bboxes);

    std::cout << "Detections after NMS: " << bboxes.size() << std::endl;
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>" << std::endl;

    delete points_data;
}

/**
 * @name: 
 * @description: Briefly describe the function of your function
 * @param {int} argc
 * @param {char} *
 * @return {*}
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arbe");
    ros::NodeHandle nh("~");

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/arbe_msg");

    // 订阅通道
    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);

    // 各种结果的发布通道
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("bboxes", 100);
    pub_bbox = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("arbe_det_bboxes", 100);

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
