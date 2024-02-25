#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#define MAX_RADAR_NUMS          (40000)
#define RADAR_FEATURES_NUMS     (5)

struct radarPoint
{
    float x;
    float y;
    float z;
    float rcs;
    float doppler;
};

struct RadarPointCloudType
{
  PCL_ADD_POINT4D      // x,y,z position in [m]
  PCL_ADD_INTENSITY;
  union
    {
      struct
      {
        float doppler;
      };
      float data_c[4];
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16; // 强制SSE填充以正确对齐内存

struct Bndbox {
    float x; 
    float y;
    float z;
    float l;
    float w;
    float h;
    float vx;
    float vy;
    float rt;
    int id;
    float score;
    int type;
    Bndbox(){};
    Bndbox(float x_, float y_, float z_, float l_, float w_, float h_, float vx_, float vy_, float rt_, int id_, float score_, int type_)
        : x(x_), y(y_), z(z_), l(l_), w(w_), h(h_), vx(vx_), vy(vy_), rt(rt_), id(id_), score(score_), type(type_) {}
};
