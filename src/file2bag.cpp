#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

// ros 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

// custom
#include "../include/common.h"

#include "../include/msg/ArbeRadarPoint.h"
#include "../include/msg/ArbeRadarMsg.h"

using std::filesystem::directory_iterator; // SUPPORTED FROM C++17

// ros可视化
ros::Publisher point_radar_pub;
ros::Publisher point_lidar_pub;
ros::Publisher marker_array_pub_;

ros::Publisher arbe_msg_pub;
ros::Publisher arbe_pc_pub;

image_transport::Publisher img_pub;

float arbe2lidar[4 * 3] = {0.9981128011677526, 0.05916115557244023, 0.016455814060541557,
                           0.07800451346438697, -0.059503891609836816, 0.9980033119043885,
                           -0.021181980812864695, 2.214080041485726, 0.015169806470300943,
                           0.022121191179620064, 0.9996402002082792, -1.6030740415943632};

/**
 * @names:
 * @description: 文件按照时间排序
 * @return {*}
 */
bool sortFileByName(std::string &a, std::string &b)
{
  double aa = std::atof(a.c_str());
  double bb = std::atof(b.c_str());

  return (aa < bb);
}

int getBinSize(const std::string path)
{
  std::ifstream infile(path, std::ifstream::binary);

  infile.seekg(0, infile.end);
  int size = infile.tellg();
  infile.seekg(0, infile.beg);
  infile.close();
  return size;
}

void readBin(std::string path, char *buf, int size)
{
  std::ifstream infile(path, std::ifstream::binary);

  infile.read(static_cast<char *>(buf), size);
  infile.close();
}

/**
 * @name:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void readRadarDataFromFiles(const std::string filePath,
                            std::vector<radarPoint> &radar_meas)
{
  DualRadar::ArbeRadarMsg arbe_msg;
  DualRadar::ArbeRadarPoint arbe_pc;

  radar_meas.clear(); // clear history

  float buf[MAX_RADAR_NUMS * RADAR_FEATURES_NUMS];

  // 获取byte size
  int data_size = getBinSize(filePath);

  // std::cout << "data size:[ " << std::to_string(data_size) << "]" << std::endl;
  data_size = (data_size > (MAX_RADAR_NUMS * RADAR_FEATURES_NUMS * sizeof(float))) ? (MAX_RADAR_NUMS * RADAR_FEATURES_NUMS * sizeof(float)) : data_size;

  std::cout << "data size:[ " << std::to_string(data_size) << "]" << std::endl;
  std::cout << "point size:[ " << std::to_string(data_size / RADAR_FEATURES_NUMS / sizeof(float)) << "]" << std::endl;

  // 读取文件数据
  readBin(filePath, (char *)buf, data_size);
  // std::cout << "Read Data Done ! " << std::endl;

  // 转换并获得点云信息
  float *data_prt = (float *)&buf[0];
  int radar_nums = data_size / RADAR_FEATURES_NUMS / sizeof(float);
  float tmpXYZ[3];

  pcl::PointCloud<pcl::PointXYZ> testcloud; // point cloud msg
  testcloud.width = radar_nums;
  testcloud.height = 1;
  testcloud.points.resize(testcloud.width * testcloud.height);

  radarPoint newRadarPc;
  for (int i = 0; i < radar_nums; i++)
  {
    tmpXYZ[0] = (*data_prt);
    data_prt++;
    tmpXYZ[1] = (*data_prt);
    data_prt++;
    tmpXYZ[2] = (*data_prt);
    data_prt++;

    // 转换之lidar坐标系
    newRadarPc.x = tmpXYZ[0] * arbe2lidar[0] + tmpXYZ[1] * arbe2lidar[1] + tmpXYZ[2] * arbe2lidar[2] + arbe2lidar[3];
    newRadarPc.y = tmpXYZ[0] * arbe2lidar[4] + tmpXYZ[1] * arbe2lidar[5] + tmpXYZ[2] * arbe2lidar[6] + arbe2lidar[7];
    newRadarPc.z = tmpXYZ[0] * arbe2lidar[8] + tmpXYZ[1] * arbe2lidar[9] + tmpXYZ[2] * arbe2lidar[10] + arbe2lidar[11];

    newRadarPc.rcs = (*data_prt);
    data_prt++;
    newRadarPc.doppler = (*data_prt);
    data_prt++;

    radar_meas.push_back(newRadarPc);

    arbe_pc.id = i;
    arbe_pc.x = tmpXYZ[0];
    arbe_pc.y = tmpXYZ[1];
    arbe_pc.z = tmpXYZ[2];
    arbe_pc.rcs = newRadarPc.rcs;
    arbe_pc.doppler = newRadarPc.doppler;
    arbe_msg.points.push_back(arbe_pc);

    testcloud.points[i].x = tmpXYZ[0];
    testcloud.points[i].y = tmpXYZ[1];
    testcloud.points[i].z = tmpXYZ[2];

    // std::cout << arbe_pc.rcs << std::endl;

  }

  std::cout << arbe_pc.rcs << std::endl;

  sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(testcloud, laserCloudTemp);
  laserCloudTemp.header.frame_id = "map";
  arbe_pc_pub.publish(laserCloudTemp); // publish

  // std::cout << "Get Radar Points:[ " << std::to_string(radar_meas.size()) << " ]" << std::endl;

  // fabu
  arbe_msg.header.frame_id = "map";
  arbe_msg.point_num = radar_nums;
  arbe_msg.points.push_back(arbe_pc);

  arbe_msg_pub.publish(arbe_msg);
}

void pubPoints2ROS(std::vector<radarPoint> &radar_meas)
{
  pcl::PointCloud<pcl::PointXYZ> testcloud; // point cloud msg
  testcloud.width = radar_meas.size();
  testcloud.height = 1;
  testcloud.points.resize(testcloud.width * testcloud.height);

  for (int i = 0; i < radar_meas.size(); i++)
  {
    testcloud.points[i].x = radar_meas.at(i).x;
    testcloud.points[i].y = radar_meas.at(i).y;
    testcloud.points[i].z = radar_meas.at(i).z;
    // testcloud.points[i].z = radar_meas.at(i).rcs;
  }

  sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(testcloud, laserCloudTemp);
  laserCloudTemp.header.frame_id = "map";
  point_radar_pub.publish(laserCloudTemp); // publish
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readBinFile(const std::string &lidar_file)
{
  std::fstream input(lidar_file, std::ios::in | std::ios::binary);
  if (!input.good())
  {
    std::cerr << "Could not read file: " << lidar_file << std::endl;
    exit(EXIT_FAILURE);
  }
  input.seekg(0, std::ios::beg);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
  int i;
  float uesless_data = 0;
  for (i = 0; input.good() && !input.eof(); i++)
  {
    pcl::PointXYZI point;
    input.read((char *)&point.x, 3 * sizeof(float));
    input.read((char *)&point.intensity, sizeof(float));
    input.read((char *)&uesless_data, 2 * sizeof(float)); //
    point.intensity *= 255;
    pointCloud->push_back(point);
  }
  input.close();

  // cout << "Read bin file from [" << input_dir << "]: "<< i << " points"<< endl;
  return pointCloud;
}

void pubLidar2ROS(std::string lidar_file)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr testcloud; // point cloud msg
  testcloud = readBinFile(lidar_file);

  sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(*testcloud, laserCloudTemp);

  laserCloudTemp.header.frame_id = "map";

  point_lidar_pub.publish(laserCloudTemp); // publish
}

void pubImage2ROS(std::string img_file)
{
  cv::Mat image = cv::imread(img_file);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  msg->header.frame_id = "map";
  img_pub.publish(msg);
}

void radarExpDemo(void)
{
  std::vector<radarPoint> radar_meas;
  // std::string folder_path = "/home/charles/code/dataset/dual_radar/testing_arbe/arbe/";
  // std::string image_path = "/home/charles/code/dataset/dual_radar/image/";
  // std::string lidar_path = "/home/charles/code/dataset/dual_radar/robosense/";

  // dual_radar dataset path

  const std::string base_path_ = "/data/chenghao/private/data/all_data/";

  std::string arbe_path = base_path_ + "/arbe/";
  std::string image_path = base_path_ + "/image/";
  std::string lidar_path = base_path_ + "/velodyne/";

  // 遍历文件夹并sort文件
  std::string file_path;
  std::vector<std::string> file_list;
  for (auto &file : directory_iterator(arbe_path))
  {
    std::string sub_file = file.path().stem().string();
    file_list.push_back(sub_file);
  }

  // 文件按照序号排序
  std::sort(file_list.begin(), file_list.end(), sortFileByName);

  // 遍历读取所有文件
  for (auto &file : file_list)
  {
    // 从点云文件读取雷达点云信息
    readRadarDataFromFiles((arbe_path + file + ".bin"), radar_meas);

    // 发布图像信息
    pubImage2ROS((image_path + file + ".png"));

    // 发布激光点云信息
    pubLidar2ROS((lidar_path + file + ".bin"));

    ros::Duration(0.05).sleep();

  }
}

int main(int argc, char **argv)
{
  std::cout << "---- It's a 4D Radar Demo----" << std::endl;

  ros::init(argc, argv, "map");
  ros::NodeHandle nh;

  arbe_msg_pub = nh.advertise<DualRadar::ArbeRadarMsg>("arbe_msg", 1);
  arbe_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("arbe_pc", 1);
  point_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("lidar", 1);
  // marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("bboxes", 100);

  image_transport::ImageTransport it(nh);
  img_pub = it.advertise("camera/image", 1);

  radarExpDemo();

  std::cout << "---- radarExpDemo Done !---- " << std::endl;

  ros::spin();
  return 0; // 返回值。
}