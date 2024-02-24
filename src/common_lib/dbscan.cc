
#include "dbscan.h"

#include <fstream>
#include <iostream>

using namespace DBSCAN;

bool showInfo = false;
std::vector<size_t> pointMap[GriDSize_Lat][GriDSize_Long];

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void DBSCAN::Proc(std::vector<Point4DBSCAN> &pointSet,
                  std::vector<std::vector<size_t>> &clusterSet)
{
  size_t minNum = 0;
  std::vector<size_t> clusterMember;

  clusterSet.clear();

  // 将点云进行栅格索引映射
  GridMappingPoint(pointSet);

  for (size_t n = 0; n < pointSet.size(); n++)
  {
    // 聚类入口
    ScanPoints(n, pointSet, &clusterMember, &minNum);

    if (clusterMember.size() >= minNum) // 是否满足最低个数限制
    {
      clusterSet.push_back(clusterMember);
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<Point4DBSCAN>} &pointSet
 * @return {*}
 */
void DBSCAN::GridMappingPoint(std::vector<Point4DBSCAN> &pointSet)
{
  size_t curLat, curLong;
  size_t tempIDX;

  for (size_t i1 = 0; i1 < GriDSize_Lat; i1++)
  {
    for (size_t i2 = 0; i2 < GriDSize_Long; i2++)
    {
      pointMap[i1][i2].clear();
    }
  }

  for (size_t i = 0; i < pointSet.size(); i++)
  {
    size_t latIdx = (pointSet[i].PointInfo.DistLat + MAX_Lat) / Grid_Reso;
    size_t longIdx = pointSet[i].PointInfo.DistLong / Grid_Reso;

    size_t scanLen = 0.0;

    // 点云横纵向初始扫描位置
    curLat = (pointSet[i].PointInfo.DistLat + MAX_Lat) / Grid_Reso;
    curLong = pointSet[i].PointInfo.DistLong / Grid_Reso;

    scanLen = pointSet[i].DBSCAN_para.Search_R / Grid_Reso + 1U;

    tempIDX = curLat + scanLen;
    if (tempIDX >= GriDSize_Lat)
    {
      tempIDX = (GriDSize_Lat - 1);
    }
    pointSet[i].PointInfo.scanLat[1] = tempIDX;

    tempIDX = curLat - scanLen;
    if (tempIDX <= 0)
    {
      tempIDX = 0;
    }
    pointSet[i].PointInfo.scanLat[0] = tempIDX;

    tempIDX = curLong + scanLen;
    if (tempIDX >= GriDSize_Long)
    {
      tempIDX = (GriDSize_Long - 1);
    }
    pointSet[i].PointInfo.scanLong[1] = tempIDX;

    tempIDX = curLong - scanLen;
    if (tempIDX <= 0)
    {
      tempIDX = 0;
    }
    pointSet[i].PointInfo.scanLong[0] = tempIDX;

    pointMap[latIdx][longIdx].push_back(i);
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void DBSCAN::ScanPoints(size_t start_Idx,
                        std::vector<Point4DBSCAN> &pointSet,
                        std::vector<size_t> *scanResult,
                        size_t *minNumer)
{
  std::vector<size_t> memberIdx;

  scanResult->clear();
  scanResult->push_back(start_Idx);

  // 无效点云
  if (pointSet[start_Idx].PointInfo.valid == false)
  {
    *minNumer = 255;
    return;
  }

  pointSet[start_Idx].PointInfo.valid = false;

  size_t tempminNumer = 255;

  for (size_t idx = 0; idx < scanResult->size(); idx++)
  {
    size_t ptsNum = 0;
    Point4DBSCAN *point2 = &pointSet[scanResult->at(idx)];

    if (point2->DBSCAN_para.minPts < tempminNumer)
    {
      tempminNumer = point2->DBSCAN_para.minPts;
    }

    // 遍历该点云左右和上下扫描范围
    for (size_t n1 = point2->PointInfo.scanLat[0]; n1 <= point2->PointInfo.scanLat[1]; n1++)
    {
      for (size_t n2 = point2->PointInfo.scanLong[0]; n2 <= point2->PointInfo.scanLong[1]; n2++)
      {
        // 对应扫描位置是否存在点云
        for (auto &idx2 : pointMap[n1][n2])
        {
          if (idx2 == scanResult->at(idx))
          {
            continue;
          }

          bool matchFlag = false;

          Point4DBSCAN *point3 = &pointSet[idx2];

          if (point3->PointInfo.valid == true)
          {
            // 计算两点之间的距离
            float diff_lat = fabs(point2->PointInfo.DistLat - point3->PointInfo.DistLat);
            float diff_long = fabs(point2->PointInfo.DistLong - point3->PointInfo.DistLong);
            float diff_height = fabs(point2->PointInfo.DisHeight - point3->PointInfo.DisHeight);
            float diff_range = sqrtf(pow(diff_lat, 2.0) + pow(diff_long, 2.0) + pow(diff_height, 2.0));

            // 计算多普勒速度差
            float diff_V = fabs(point2->PointInfo.V - point3->PointInfo.V);

            float diff_Azi = fabs(point2->PointInfo.Azi - point3->PointInfo.Azi);

            float range_gate = (point2->DBSCAN_para.Search_R >= point3->DBSCAN_para.Search_R) ? point2->DBSCAN_para.Search_R : point3->DBSCAN_para.Search_R;

            // 欧式距离差值和多普勒速度差值
            if ((diff_range < range_gate) && (diff_V < 1.0))
            {
              matchFlag = true;
            }

            if (matchFlag == true)
            {
              ptsNum++;
              point3->PointInfo.valid = false;
              scanResult->push_back(idx2);
            }
          }
        }
      }
    }
  }

  *minNumer = tempminNumer;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<Point4DBSCAN>} &pointSet
 * @return {*}
 */
void DBSCAN::ComputePara(std::vector<Point4DBSCAN> &pointSet)
{
  // 根据不同的条件，决定该点的SCAN参数
  for (auto &point : pointSet)
  {
    point.DBSCAN_para.Search_R = 1.5F;

    if (fabs(point.PointInfo.V) > 5.0F)
    {
      point.DBSCAN_para.Search_R = 2.5F;
    }

    point.DBSCAN_para.minPts = 2;

    point.DBSCAN_para.pointType = 255;
  }
}