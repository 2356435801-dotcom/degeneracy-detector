#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>

/**
 * 一帧激光扫描数据
 */
struct ScanFrame
{
    /// 时间戳
    ros::Time stamp;

    /// 二维点云
    std::vector<Eigen::Vector2d> points;
};

/**
 * 协方差计算结果
 */
struct CovarianceResult
{
    /// 点云质心
    Eigen::Vector2d centroid =
        Eigen::Vector2d::Zero();

    /// 协方差矩阵
    Eigen::Matrix2d covariance =
        Eigen::Matrix2d::Zero();

    /// 特征值
    Eigen::Vector2d eigen_values =
        Eigen::Vector2d::Zero();

    /// 特征向量
    Eigen::Matrix2d eigen_vectors =
        Eigen::Matrix2d::Identity();
};

/**
 * 退化分析结果
 */
struct DegeneracyResult
{
    /// 是否退化
    bool degenerate = false;

    /// 条件数
    double score = 0.0;

    /// 最大特征值
    double lambda_max = 0.0;

    /// 最小特征值
    double lambda_min = 0.0;

    /// 退化方向
    Eigen::Vector2d direction =
        Eigen::Vector2d::Zero();
};

#endif
