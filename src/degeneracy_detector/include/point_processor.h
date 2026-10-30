#ifndef POINT_PROCESSOR_H
#define POINT_PROCESSOR_H

#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <vector>

#include "data_type.h"

/**
 * @brief 点云预处理模块
 *
 * 功能：
 * 1. LaserScan → ScanFrame
 * 2. NaN过滤
 * 3. Inf过滤
 * 4. 距离过滤
 * 5. 后续可扩展Voxel滤波、离群点滤波等
 */
class PointProcessor
{
public:

    PointProcessor();

    /**
     * @brief LaserScan转换为ScanFrame
     *
     * @param scan 原始LaserScan
     * @param frame 输出ScanFrame
     * @return 是否成功
     */
    bool convertLaserScan(
            const sensor_msgs::LaserScan& scan,
            ScanFrame& frame);

    /**
     * @brief 设置距离过滤范围
     */
    void setRangeLimit(
            double min_range,
            double max_range);

    /**
     * @brief 获取最小距离
     */
    double minRange() const;

    /**
     * @brief 获取最大距离
     */
    double maxRange() const;

private:

    double min_range_;

    double max_range_;
};

#endif
