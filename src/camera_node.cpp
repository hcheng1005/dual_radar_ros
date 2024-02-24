// sys
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

// custom


void callbackCloud(const sensor_msgs::ImagePtr &cam_msg)
{
    ROS_INFO_STREAM("\033[1;32m" << "Rec CAM_msg: " << "\033[0m");
}


int main(int argc, char**argv) {

    ros::init(argc, argv, "camera");
    ros::NodeHandle nn("~");

    std::string cloud_topic;
    nn.param<std::string>("camera_topic", cloud_topic, "/camera/image");
    ros::Subscriber sub_cloud = nn.subscribe(cloud_topic, 100, callbackCloud);
    
    ros::spin();

    return 0;
}