#ifndef DEGENERACY_ENGINE_H
#define DEGENERACY_ENGINE_H

#include <Eigen/Dense>

#include <vector>

#include "adaptive_weight.h"
#include "covariance.h"
#include "data_type.h"
#include "degeneracy_analyzer.h"
#include "degeneracy_config.h"
#include "sliding_window.h"

/**
 * @brief 退化检测算法核心引擎
 *
 * Engine不依赖：
 *
 *     sensor_msgs::LaserScan
 *     ROS Publisher
 *     ROS Subscriber
 *
 * Engine只接收已经转换好的二维点。
 *
 * 数据流：
 *
 *     points
 *       ↓
 *     ScanFrame
 *       ↓
 *     SlidingWindow
 *       ↓
 *     Covariance
 *       ↓
 *     DegeneracyAnalyzer
 *       ↓
 *     AdaptiveWeight
 */
class DegeneracyEngine
{
public:

    /**
     * @brief 默认构造
     */
    DegeneracyEngine();

    /**
     * @brief 使用指定配置构造
     */
    explicit DegeneracyEngine(
            const DegeneracyConfig& config);

    /**
     * @brief 设置完整配置
     */
    void setConfig(
            const DegeneracyConfig& config);

    /**
     * @brief 获取当前配置
     */
    const DegeneracyConfig& config() const;

    /**
     * @brief 更新一帧点云并执行退化检测
     *
     * @param points 当前帧二维点云
     * @param stamp 当前帧时间戳
     * @param merged_frame 滑动窗口融合后的点云
     * @param covariance_result 协方差计算结果
     * @param degeneracy_result 退化分析结果
     * @param weight_result 自适应权重结果
     *
     * @return true 本次计算成功
     * @return false 当前数据无法完成计算
     */
    bool update(
            const std::vector<Eigen::Vector2d>& points,
            const ros::Time& stamp,
            ScanFrame& merged_frame,
            CovarianceResult& covariance_result,
            DegeneracyResult& degeneracy_result,
            WeightResult& weight_result);

    /**
     * @brief 获取当前滑动窗口帧数
     */
    size_t frameCount() const;

    /**
     * @brief 获取滑动窗口大小
     */
    size_t windowSize() const;

private:

    DegeneracyConfig config_;

    SlidingWindow sliding_window_;

    CovarianceCalculator covariance_calculator_;

    DegeneracyAnalyzer analyzer_;

    AdaptiveWeightCalculator weight_calculator_;

    ScanFrame current_frame_;
};

#endif
