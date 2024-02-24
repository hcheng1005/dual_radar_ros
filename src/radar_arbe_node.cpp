// sys
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include "../include/msg/ArbeRadarPoint.h"
#include "../include/msg/ArbeRadarMsg.h"

// custom


void callbackCloud(const radarExp::ArbeRadarMsg &arbe_msg)
{
    ROS_INFO_STREAM("\033[1;32m" << "Rec Arbe_msg: " << "\033[0m");
}


int main(int argc, char**argv) {

    ros::init(argc, argv, "arbe");
    ros::NodeHandle nn("~");

    std::string cloud_topic;
    nn.param<std::string>("cloud_topic", cloud_topic, "/arbe_msg");
    ros::Subscriber sub_cloud = nn.subscribe(cloud_topic, 100, callbackCloud);
    
    ros::spin();

    return 0;
}