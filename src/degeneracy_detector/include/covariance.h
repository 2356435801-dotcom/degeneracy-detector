#ifndef COVARIANCE_H
#define COVARIANCE_H

#include <Eigen/Dense>

#include "data_type.h"

/**
 * @brief 协方差计算器
 *
 * 输入：
 *      ScanFrame
 *
 * 输出：
 *      CovarianceResult
 *
 * 仅负责数学计算，不保存任何状态。
 */
class CovarianceCalculator
{
public:

    CovarianceCalculator() = default;

    /**
     * @brief 计算点云协方差及PCA
     *
     * @param frame 输入点云
     * @param result 输出结果
     * @return true 成功
     * @return false 点数不足
     */
    bool compute(
            const ScanFrame& frame,
            CovarianceResult& result) const;
};

#endif
