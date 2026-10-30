#ifndef CARTOGRAPHER_ADAPTER_H
#define CARTOGRAPHER_ADAPTER_H

#include <Eigen/Dense>

#include <vector>

namespace cartographer
{
namespace sensor
{
class PointCloud;
}
}

/**
 * @brief Cartographer点云适配器
 *
 * 负责将Cartographer内部PointCloud
 * 转换成退化检测模块使用的二维Eigen点集。
 *
 * 设计原则：
 *
 * Cartographer
 *     ↓
 * sensor::PointCloud
 *     ↓
 * CartographerAdapter
 *     ↓
 * std::vector<Eigen::Vector2d>
 *
 * DegeneracyEngine不直接依赖Cartographer。
 */
class CartographerAdapter
{
public:

    CartographerAdapter();

    /**
     * @brief 将Cartographer PointCloud转换为二维点云
     *
     * @param point_cloud Cartographer内部点云
     * @param points 输出二维点集
     *
     * @return true 转换成功
     * @return false 点云为空
     */
    bool convert(
            const cartographer::sensor::PointCloud& point_cloud,
            std::vector<Eigen::Vector2d>& points) const;
};

#endif
