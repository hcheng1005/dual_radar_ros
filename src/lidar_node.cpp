// sys
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// custom


void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    ROS_INFO_STREAM("\033[1;32m" << "Rec Lidar_msg: " << "\033[0m");
}


int main(int argc, char**argv) {

    ros::init(argc, argv, "lidar");
    ros::NodeHandle nn("~");

    std::string cloud_topic;
    nn.param<std::string>("lidar_topic", cloud_topic, "/lidar");
    ros::Subscriber sub_cloud = nn.subscribe(cloud_topic, 100, callbackCloud);
    
    ros::spin();

    return 0;
}