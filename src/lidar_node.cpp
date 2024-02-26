// sys
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "./pointpillars/pointpillars.h"

// custom

PointPillars *PointPillars_ = nullptr;

int Txt2Arrary(float *&points_array, string file_name, int num_feature = 4)
{
    ifstream InFile;
    InFile.open(file_name.data());
    assert(InFile.is_open());

    vector<float> temp_points;
    string c;

    while (!InFile.eof())
    {
        InFile >> c;

        temp_points.push_back(atof(c.c_str()));
    }
    points_array = new float[temp_points.size()];
    for (int i = 0; i < temp_points.size(); ++i)
    {
        points_array[i] = temp_points[i];
    }

    InFile.close();
    return temp_points.size() / num_feature;
    // printf("Done");
};

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    ROS_INFO_STREAM("\033[1;32m"
                    << "Rec Lidar_msg: "
                    << "\033[0m" << " t:" << cloud_msg->header.stamp);

    std::string file_name = "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars/testdata/nuscenes_10sweeps_points.txt";
    float *points_array;
    int in_num_points;
    in_num_points = Txt2Arrary(points_array, file_name, 5);

    std::vector<float> out_detections;
    std::vector<int> out_labels;
    std::vector<float> out_scores;

    PointPillars_->DoInference(points_array, in_num_points, &out_detections, &out_labels, &out_scores);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar");
    ros::NodeHandle nn("~");

    std::string cloud_topic;
    nn.param<std::string>("lidar_topic", cloud_topic, "/lidar");
    ros::Subscriber sub_cloud = nn.subscribe(cloud_topic, 100, callbackCloud);

    PointPillars_ = new PointPillars(0.1,
                                     0.3,
                                     false,
                                     "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars/model/cbgs_pp_multihead_pfe.trt",
                                     "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars/model/cbgs_pp_multihead_backbone.trt",
                                     "/data/chenghao/private/pp_ros_ws/src/dual_radar_ros/src/pointpillars/cfgs/cbgs_pp_multihead.yaml");

    ros::spin();

    return 0;
}