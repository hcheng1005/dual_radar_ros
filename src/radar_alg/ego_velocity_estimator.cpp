
#define PCL_NO_PRECOMPILE

#include <algorithm>
#include <iostream>
#include <random>
#include <sstream>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "ego_velocity_estimator.h"



using namespace std;
using namespace rio;

static RadarPointCloudType toRadarPointCloudType(const Vector11 &item, const RadarEgoVelocityEstimatorIndices &idx)
{
    RadarPointCloudType point;
    point.x = item[idx.x_r];
    point.y = item[idx.y_r];
    point.z = item[idx.z_r];
    point.doppler = -item[idx.doppler];
    point.intensity = item[idx.snr_db];
    return point;
}

// bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2 &radar_scan_msg,
//                                          Vector3 &v_r,
//                                          Vector3 &sigma_v_r)
// {
//     sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
//     return estimate(radar_scan_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg);
// }

bool RadarEgoVelocityEstimator::estimate(const std::vector<radarPoint> &radar_scan,
                                                Vector3 &v_r,
                                                Vector3 &sigma_v_r,
                                                sensor_msgs::PointCloud2 &inlier_radar_msg,
                                                sensor_msgs::PointCloud2 &outlier_radar_msg)
{
    // auto radar_scan(new pcl::PointCloud<RadarPointCloudType>);
    auto radar_scan_inlier(new pcl::PointCloud<RadarPointCloudType>);  // static objects 静态点云
    auto radar_scan_outlier(new pcl::PointCloud<RadarPointCloudType>); // 非静态点

    bool success = false;
    //   pcl::fromROSMsg(radar_scan_msg, *radar_scan);
    if (1) // pcl2msgToPcl(radar_scan_msg, *radar_scan)
    {
        std::vector<Vector11> valid_targets;
        for (uint i = 0; i < radar_scan.size(); ++i)
        {
            const auto target = radar_scan.at(i);

            // 点云距离
            const double r = Vector3(target.x, target.y, target.z).norm();

            // 点云水平角度和俯仰角计算
            Real azimuth = std::atan2(target.x, target.y);
            Real elevation = std::atan2(target.z, std::sqrt(target.x * target.x + target.y * target.y));

            // std::cout << azimuth / M_PI * 180.0 << ", " << elevation / M_PI * 180.0 << std::endl;

            // 点云筛选
            if (r > config_.min_dist && r < config_.max_dist && 1 &&
                std::fabs(azimuth) < (config_.azimuth_thresh_deg / 180.0 * M_PI) &&
                std::fabs(elevation) < (config_.elevation_thresh_deg / 180.0 * M_PI))
            {
                // 构造新点云信息（xyz,i,v,r,azi,elev, target.x / r, target.y / r, target.z / r）
                Vector11 v_pt; // features of a point
                // v_pt << target.x, target.y, target.z, target.rcs, -target.doppler * config_.doppler_velocity_correction_factor,
                //     r, azimuth, elevation, target.x / r, target.y / r, target.z / r;
                v_pt << target.x, target.y, target.z, target.rcs, -target.doppler * 1.0,
                    r, azimuth, elevation, target.x / r, target.y / r, target.z / r;

                valid_targets.emplace_back(v_pt);
            }
        }

        if (valid_targets.size() > 2)
        {
            ROS_INFO("Valid_targets in radar_scan (%ld)", valid_targets.size());
            // check for zero velocity
            std::vector<double> v_dopplers;
            for (const auto &v_pt : valid_targets)
            {
                v_dopplers.emplace_back(std::fabs(v_pt[idx_.doppler]));
            }

            const size_t n = v_dopplers.size() * (1.0 - config_.allowed_outlier_percentage);

            // 点云排序
            // std::nth_element用法 https://blog.csdn.net/xiaoquantouer/article/details/51591140
            std::nth_element(v_dopplers.begin(), v_dopplers.begin() + n, v_dopplers.end());
            const auto median = v_dopplers[n];

            if (median < config_.thresh_zero_velocity) // 车辆静止
            {
                // ROS_INFO_STREAM_THROTTLE(0.5, kPrefix << "Zero velocity detected!");
                v_r = Vector3(0, 0, 0);
                sigma_v_r =
                    Vector3(config_.sigma_zero_velocity_x, config_.sigma_zero_velocity_y, config_.sigma_zero_velocity_z);
                for (const auto &item : valid_targets)
                    if (std::fabs(item[idx_.doppler]) < config_.thresh_zero_velocity)
                        radar_scan_inlier->push_back(toRadarPointCloudType(item, idx_));
                success = true;
            }
            else
            {
                // 参考论文 https://www.researchgate.net/profile/Dominik-Kellner/publication/269332200_Instantaneous_ego-motion_estimation_using_Doppler_radar/links/57742a2508ae1b18a7de4150/Instantaneous-ego-motion-estimation-using-Doppler-radar.pdf
                // LSQ velocity estimation
                Matrix radar_data(valid_targets.size(), 4); // rx, ry, rz, v
                uint idx = 0, idx_o = 0;
                for (const auto &v_pt : valid_targets)
                {
                    // 使用的是归一化距离信息
                    radar_data.row(idx++) = Vector4(v_pt[idx_.normalized_x], v_pt[idx_.normalized_y], v_pt[idx_.normalized_z], v_pt[idx_.doppler]);
                }

                if (config_.use_ransac) // 默认使用随机采样一致性
                {
                    std::vector<uint> inlier_idx_best;
                    std::vector<uint> outlier_idx_best;

                    // 调用RANSAC算法进行点云分离
                    success = solve3DFullRansac(radar_data, v_r, sigma_v_r, inlier_idx_best, outlier_idx_best);

                    std::cout << v_r.transpose() << std::endl;

                    // Insert inlier points to the cloud
                    for (const auto &idx : inlier_idx_best)
                    {
                        radar_scan_inlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
                    }

                    for (const auto &idx : outlier_idx_best)
                    {
                        radar_scan_outlier->push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
                    }
                }
                else
                {
                    for (const auto &item : valid_targets)
                        radar_scan_inlier->push_back(toRadarPointCloudType(item, idx_));
                    success = solve3DFull(radar_data, v_r, sigma_v_r);
                }
            }
        }
        else
        {
            ROS_INFO("To small valid_targets (< 2) in radar_scan (%ld)", radar_scan.size());
        }

        ROS_INFO("inlier (%ld) outlier (%ld)", radar_scan_inlier->size(), radar_scan_outlier->size());

        radar_scan_inlier->height = 1;
        radar_scan_inlier->width = radar_scan_inlier->size();



        // pcl::PCLPointCloud2 tmp;
        // pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_inlier, tmp);
        // pcl_conversions::fromPCL(tmp, inlier_radar_msg);
        // inlier_radar_msg.header = radar_scan_msg.header;

        // radar_scan_outlier->height = 1;
        // radar_scan_outlier->width = radar_scan_outlier->size();

        // pcl::PCLPointCloud2 tmp_o;
        // pcl::toPCLPointCloud2<RadarPointCloudType>(*radar_scan_outlier, tmp_o);
        // pcl_conversions::fromPCL(tmp_o, outlier_radar_msg);
        // outlier_radar_msg.header = radar_scan_msg.header;
    }

    if(success)
      ROS_INFO("Ego Velocity estimation Successful");
    else
      ROS_INFO("Ego Velocity estimation Failed");

    return success;
}

/**
 * @name: solve3DFullRansac
 * @description: 调用RANSAC分离动静点
 * @param {Matrix} &radar_data
 * @param {Vector3} &v_r
 * @param {Vector3} &sigma_v_r
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
bool RadarEgoVelocityEstimator::solve3DFullRansac(const Matrix &radar_data,
                                                  Vector3 &v_r,
                                                  Vector3 &sigma_v_r,
                                                  std::vector<uint> &inlier_idx_best,
                                                  std::vector<uint> &outlier_idx_best)
{
    Matrix H_all(radar_data.rows(), 3);
    H_all.col(0) = radar_data.col(0);
    H_all.col(1) = radar_data.col(1);
    H_all.col(2) = radar_data.col(2);
    const Vector y_all = radar_data.col(3);

    std::vector<uint> idx(radar_data.rows());
    for (uint k = 0; k < radar_data.rows(); ++k)
        idx[k] = k;

    std::random_device rd;
    std::mt19937 g(rd());

    if (radar_data.rows() >= config_.N_ransac_points)
    {
        for (uint k = 0; k < ransac_iter_; ++k) // 迭代n次，最终取动静点最多case作为最终结果
        {
            // 洗牌（无序化处理）
            // std::shuffle https://blog.csdn.net/albertsh/article/details/124562399
            std::shuffle(idx.begin(), idx.end(), g);
            Matrix radar_data_iter;
            radar_data_iter.resize(config_.N_ransac_points, 4);

            for (uint i = 0; i < config_.N_ransac_points; ++i)
                radar_data_iter.row(i) = radar_data.row(idx.at(i));

            bool rtn = solve3DFull(radar_data_iter, v_r, sigma_v_r, false);

            if (rtn == false)
                ROS_INFO("Failure at solve3DFullRansac() 1");
            if (rtn)
            {
                const Vector err = (y_all - H_all * v_r).array().abs(); // 计算所有点与“静止速度”的差值
                std::vector<uint> inlier_idx;
                std::vector<uint> outlier_idx;
                for (uint j = 0; j < err.rows(); ++j)
                {
                    // ROS_INFO("Error: %f",err(j));
                    // 根据设定阈值进行动静点分离
                    if (err(j) < config_.inlier_thresh)
                        // find inlier points
                        inlier_idx.emplace_back(j);
                    else
                        outlier_idx.emplace_back(j);
                }
                // if too small number of inlier detected, the error is too high, so regard outlier as inlier
                if (float(outlier_idx.size()) / (inlier_idx.size() + outlier_idx.size()) > 0.05)
                {
                    inlier_idx.insert(inlier_idx.end(), outlier_idx.begin(), outlier_idx.end());
                    outlier_idx.clear();
                    // outlier_idx.swap(std::vector<uint>());
                }

                // ROS_INFO("Inlier number: %ld, Outlier number: %ld, outlier Ratio: %f",
                //           inlier_idx.size(), outlier_idx.size(), float(outlier_idx.size())/(inlier_idx.size()+outlier_idx.size()));

                if (inlier_idx.size() > inlier_idx_best.size())
                {
                    inlier_idx_best = inlier_idx;
                }
                if (outlier_idx.size() > outlier_idx_best.size())
                {
                    outlier_idx_best = outlier_idx;
                }
            }
        }
    }
    else
    {
        ROS_INFO("Warning: radar_data.rows() < config_.N_ransac_points");
    }

    if (!inlier_idx_best.empty())
    {
        Matrix radar_data_inlier(inlier_idx_best.size(), 4);
        for (uint i = 0; i < inlier_idx_best.size(); ++i)
            radar_data_inlier.row(i) = radar_data.row(inlier_idx_best.at(i));

        bool rtn = solve3DFull(radar_data_inlier, v_r, sigma_v_r, true);
        if (rtn == false)
            ROS_INFO("Failure at solve3DFullRansac() 2");
        return rtn;
    }
    ROS_INFO("Failure at solve3DFullRansac() 3");
    return false;
}

bool RadarEgoVelocityEstimator::solve3DFull(const Matrix &radar_data,
                                            Vector3 &v_r,
                                            Vector3 &sigma_v_r,
                                            bool estimate_sigma)
{
    Matrix H(radar_data.rows(), 3);
    H.col(0) = radar_data.col(0);
    H.col(1) = radar_data.col(1);
    H.col(2) = radar_data.col(2);
    const Matrix HTH = H.transpose() * H;

    const Vector y = radar_data.col(3);

    Eigen::JacobiSVD<Matrix> svd(HTH);
    Real cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

    // cond > 1000, error occurs
    if (1) // std::fabs(cond) < 1.0e3
    {
        if (config_.use_cholesky_instead_of_bdcsvd)
        {
            v_r = (HTH).ldlt().solve(H.transpose() * y);
        }
        else
            v_r = H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);

        if (estimate_sigma) // 默认false
        {
            const Vector e = H * v_r - y;
            const Matrix C = (e.transpose() * e).x() * (HTH).inverse() / (H.rows() - 3);
            sigma_v_r = Vector3(C(0, 0), C(1, 1), C(2, 2));
            sigma_v_r = sigma_v_r.array();
            if (sigma_v_r.x() >= 0.0 && sigma_v_r.y() >= 0.0 && sigma_v_r.z() >= 0.)
            {
                sigma_v_r = sigma_v_r.array().sqrt();
                sigma_v_r += Vector3(config_.sigma_offset_radar_x, config_.sigma_offset_radar_y, config_.sigma_offset_radar_z);
                if (sigma_v_r.x() < config_.max_sigma_x && sigma_v_r.y() < config_.max_sigma_y &&
                    sigma_v_r.z() < config_.max_sigma_z)
                {
                    return true;
                }
            }
        }
        else
        {
            return true;
        }
    }
    // ROS_INFO("cond too large, cond = %f", cond);

    return true; // return false;
}