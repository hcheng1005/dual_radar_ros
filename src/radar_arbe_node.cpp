// sys
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include "../include/msg/ArbeRadarPoint.h"
#include "../include/msg/ArbeRadarMsg.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// custom
#include "./radar_alg/radar_alg_main.h"


ros::Publisher marker_array_pub_;

RadarExp::radarAlg *radarRuner = nullptr;

// std::mutex 


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

void callbackCloud(const radarExp::ArbeRadarMsg &arbe_msg)
{
    ROS_INFO_STREAM("\033[1;32m" << "Rec Arbe_msg: " << "\033[0m");

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


int main(int argc, char**argv) {

    ros::init(argc, argv, "arbe");
    ros::NodeHandle nn("~");

    std::string cloud_topic;
    nn.param<std::string>("cloud_topic", cloud_topic, "/arbe_msg");

    // 订阅通道
    ros::Subscriber sub_cloud = nn.subscribe(cloud_topic, 100, callbackCloud);

    // 各种结果的发布通道
    marker_array_pub_ = nn.advertise<visualization_msgs::MarkerArray>("bboxes", 100);

    // 注册算法模块
    // radarRuner = new RadarExp::radarAlg();

    // PointPillars_ = new PointPillars(0.7,
    //                                 0.1,
    //                                 false,
    //                                 "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars/model/arbe_pp_pfe.trt",
    //                                 "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars/model/arbe_pp_backbone.trt",
    //                                 "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars/cfgs/pointpillar_arbe.yaml");
    
    ros::spin();

    return 0;
}

