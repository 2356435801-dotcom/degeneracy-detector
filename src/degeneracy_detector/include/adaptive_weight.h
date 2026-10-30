#ifndef ADAPTIVE_WEIGHT_H
#define ADAPTIVE_WEIGHT_H

#include "degeneracy_config.h"

/**
 * @brief 自适应权重计算结果
 */
struct WeightResult
{
    /**
     * @brief 平移权重
     */
    double translation_weight = 0.0;

    /**
     * @brief 旋转权重
     */
    double rotation_weight = 0.0;
};

/**
 * @brief 自适应扫描匹配权重计算器
 *
 * 根据退化程度score动态调整平移权重。
 *
 * 当前策略：
 *
 * translation_weight =
 * translation_max /
 * (1 + gain * score)
 *
 * rotation_weight保持基础值。
 */
class AdaptiveWeightCalculator
{
public:

    /**
     * @brief 使用默认配置构造
     */
    AdaptiveWeightCalculator();

    /**
     * @brief 使用指定配置构造
     */
    explicit AdaptiveWeightCalculator(
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
     * @brief 根据退化score计算自适应权重
     */
    WeightResult calculate(
            double score) const;

    /**
     * @brief 设置最大平移权重
     */
    void setMaximumTranslation(
            double w);

    /**
     * @brief 设置旋转权重
     */
    void setRotationWeight(
            double w);

    /**
     * @brief 设置自适应增益
     */
    void setGain(
            double k);

private:

    DegeneracyConfig config_;
};

#endif
