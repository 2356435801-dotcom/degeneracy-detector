#include "point_processor.h"

#include <cmath>

PointProcessor::PointProcessor()
{
    min_range_ = 0.10;
    max_range_ = 30.0;
}

void PointProcessor::setRangeLimit(
        double min_range,
        double max_range)
{
    min_range_ = min_range;
    max_range_ = max_range;
}

double PointProcessor::minRange() const
{
    return min_range_;
}

double PointProcessor::maxRange() const
{
    return max_range_;
}

bool PointProcessor::convertLaserScan(
        const sensor_msgs::LaserScan& scan,
        ScanFrame& frame)
{
    // 清空上一帧数据
    frame.points.clear();

    // 保存时间戳
    frame.stamp = scan.header.stamp;

    // 预分配空间，减少vector扩容次数
    frame.points.reserve(scan.ranges.size());

    for(size_t i = 0; i < scan.ranges.size(); ++i)
    {
        double range = scan.ranges[i];

        // NaN过滤
        if(std::isnan(range))
            continue;

        // Inf过滤
        if(std::isinf(range))
            continue;

        // 最小距离过滤
        if(range < min_range_)
            continue;

        // 最大距离过滤
        if(range > max_range_)
            continue;

        double angle =
                scan.angle_min +
                i * scan.angle_increment;

        double x = range * std::cos(angle);

        double y = range * std::sin(angle);

        frame.points.emplace_back(x, y);
    }

    // 点数过少直接认为失败
    if(frame.points.size() < 10)
        return false;

    return true;
}
