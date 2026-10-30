#ifndef DEGENERACY_ANALYZER_H
#define DEGENERACY_ANALYZER_H

#include "data_type.h"
#include "degeneracy_config.h"

/**
 * @brief 退化分析器
 *
 * 输入：
 *     CovarianceResult
 *
 * 输出：
 *     DegeneracyResult
 *
 * 功能：
 *     1. 计算退化指标score
 *     2. 判断是否退化
 *     3. 提取退化方向
 */
class DegeneracyAnalyzer
{
public:

    /**
     * @brief 使用默认配置
     */
    DegeneracyAnalyzer();

    /**
     * @brief 使用指定配置
     */
    explicit DegeneracyAnalyzer(
            const DegeneracyConfig& config);

    /**
     * @brief 设置配置
     */
    void setConfig(
            const DegeneracyConfig& config);

    /**
     * @brief 获取配置
     */
    const DegeneracyConfig& config() const;

    /**
     * @brief 设置退化阈值
     *
     * 为保持与之前代码兼容而保留。
     */
    void setThreshold(
            double threshold);

    /**
     * @brief 获取当前退化阈值
     */
    double threshold() const;

    /**
     * @brief 执行退化分析
     */
    bool analyze(
            const CovarianceResult& covariance,
            DegeneracyResult& result) const;

private:

    DegeneracyConfig config_;
};

#endif
