#include "cartographer_adapter.h"

#include <cmath>

#include "cartographer/sensor/point_cloud.h"

bool CartographerAdapter::convert(
    const cartographer::sensor::PointCloud& point_cloud,
    std::vector<Eigen::Vector2d>& points) const
{
    points.clear();
    points.reserve(point_cloud.size());

    for (const auto& point : point_cloud) {

        const double x =
            static_cast<double>(point.position.x());

        const double y =
            static_cast<double>(point.position.y());

        if (!std::isfinite(x) ||
            !std::isfinite(y)) {
            continue;
        }

        points.emplace_back(x, y);
    }

    return !points.empty();
}
