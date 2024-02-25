#include "radar_alg_main.h"
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>

cv::Mat image = cv::Mat::zeros(600, 800, CV_8UC3); // 宽800，高600，3通道图像

sensor_msgs::PointCloud2 inlier_radar_msg;
sensor_msgs::PointCloud2 outlier_radar_msg;

namespace RadarExp
{
    void radarAlg::proc(std::vector<radarPoint> &radarPoints)
    {
        // 点云聚类
        radarPointsPtr = std::make_shared<std::vector<radarPoint>>(radarPoints);

        // 点云过滤
        pointsFilter(radarPointsPtr);

        EgoVelocityEstimator(radarPoints);

        // 点云聚类以及box拟合
        // getRadarClusters();

        // 跟踪算法
    }

    void radarAlg::pointsFilter(std::shared_ptr<std::vector<radarPoint>> &pointPtr)
    {
        // 遍历所有点云，删除指定范围的点云
        std::vector<radarPoint> newPoints;
        for (int i = 0; i < pointPtr->size(); i++)
        {
            auto &pc = pointPtr->at(i);

            if ((pc.z < -2.0) || (pc.z > 3.0))
            {
                continue;
            }

            newPoints.push_back(pc);
        }

        std::cout << "Before Filter: " << pointPtr->size() << " \n"
                  << "After Filter: " << newPoints.size() << std::endl;

        pointPtr = std::make_shared<std::vector<radarPoint>>(newPoints);
    }

    void radarAlg::EgoVelocityEstimator(std::vector<radarPoint> &radarPoints)
    {
        rio::Vector3 v_r, sigma_v_r;
        RadarEgoVelocityEstimatorPtr->estimate(radarPoints, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg);
    }

    /**
     * @name: getRadarClusters
     * @description: 毫米波雷达聚类算法
     * @return {*}
     */
    void radarAlg::getRadarClusters()
    {
        DBSCAN::Point4DBSCAN Point;
        std::vector<DBSCAN::Point4DBSCAN> PointSet;
        std::vector<std::vector<size_t>> clusterSet;

        image = cv::Mat::zeros(600, 800, CV_8UC3); // 宽800，高600，3通道图像

        // 点云格式转换，进行dbscan聚类
        for (uint i = 0; i < radarPointsPtr->size(); i++)
        {
            auto &radar_p = radarPointsPtr->at(i);

            // 构造聚类点云
            DBSCAN::Point4DBSCAN Point;

            Point.PointInfo.ID = i;
            Point.PointInfo.DistLat = radar_p.y;
            Point.PointInfo.DistLong = radar_p.x;
            Point.PointInfo.DisHeight = radar_p.z;

            Point.PointInfo.V = radar_p.doppler;
            Point.PointInfo.RCS = radar_p.rcs;

            Point.DBSCAN_para.Search_R = 1.0F;
            Point.DBSCAN_para.minPts = 10;
            Point.DBSCAN_para.pointType = 255;
            Point.DBSCAN_para.static_or_dyna = 0;

            Point.PointInfo.valid = true;

            PointSet.push_back(Point);

            // 添加点到点云数据
            if (VISUALIZATION)
            {
                cv::circle(image,
                           cv::Point2f((radar_p.x + 100) / 200 * 800, 600 - radar_p.y / 100 * 600),
                           3, cv::Scalar(200, 200, 200), -1);
            }
        }

        // Step 1: cluster
        DBSCAN::Proc(PointSet, clusterSet);

        std::cout << " cluster num:[ " << clusterSet.size() << " ]" << std::endl;

        getDetBox(PointSet, clusterSet);

        if (VISUALIZATION)
        {
            cv::imshow("Point Cloud Visualization", image);
            cv::waitKey(50);
        }
    }

    void radarAlg::getDetBox(const std::vector<DBSCAN::Point4DBSCAN> PointSet,
                             const std::vector<std::vector<size_t>> &clusterSet)
    {
        Bndboxs.clear();

        const double maxAngle = M_PI_2;
        const double minAngle = -M_PI_2;
        const double step = M_PI_2 / 90.0;

        int boxId = 0;
        for (const auto &subCluster : clusterSet)
        {
            Eigen::MatrixXd pointMat(2, subCluster.size());
            std::vector<float> measZ; // Z轴参数单独计算

            for (uint idx = 0; idx < subCluster.size(); idx++)
            {
                pointMat(0, idx) = radarPointsPtr->at(PointSet.at(subCluster.at(idx)).PointInfo.ID).x;
                pointMat(1, idx) = radarPointsPtr->at(PointSet.at(subCluster.at(idx)).PointInfo.ID).y;
                measZ.push_back(radarPointsPtr->at(PointSet.at(subCluster.at(idx)).PointInfo.ID).z);
            }

            // 基于L_Shape Fitting拟合box
            RotRect2D_t RotRect = L_shape_Fit_Proc(pointMat, maxAngle, minAngle, step);

            // z轴属性计算：中心位置和高度
            auto z_minmax = std::minmax(measZ.begin(), measZ.end());
            float z_center = (*z_minmax.first + *z_minmax.second) * 0.5;
            float z_height = (*z_minmax.second + *z_minmax.first) * 0.5;

            // 构造3d-bbox
            Bndbox newBox(RotRect.corner[4][0], RotRect.corner[4][1], z_center,
                          RotRect.length, RotRect.width, z_height, 0.0, 0.0, RotRect.theta, boxId, 0.0, 0);
            Bndboxs.push_back(newBox);

            boxId++;

            // std::cout << RotRect.corner[4][0] << ", "
            //         << RotRect.corner[4][1] << ", "
            //         << RotRect.length << ", "
            //         << RotRect.width << ", "
            //         << RotRect.theta / M_PI * 180.0
            //         << std::endl;

            if (VISUALIZATION)
            {
                cv::Rect rect((RotRect.corner[4][0] - RotRect.width * 0.5 + 100) / 200 * 800,
                              600 - (RotRect.corner[4][1] + RotRect.length * 0.5) / 100 * 600,
                              RotRect.width / 200 * 800,
                              RotRect.length / 100 * 600); // 定义矩形框，左上角坐标 (200, 200)，宽高 (200, 200)

                cv::rectangle(image, rect, cv::Scalar(125, 125, 125), 2); // 绿色矩形框
            }
        }
    }
}