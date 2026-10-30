#include "adaptive_weight.h"

AdaptiveWeightCalculator::
AdaptiveWeightCalculator()
    : config_()
{
}

AdaptiveWeightCalculator::
AdaptiveWeightCalculator(
        const DegeneracyConfig& config)
    : config_(config)
{
}

void AdaptiveWeightCalculator::
setConfig(
        const DegeneracyConfig& config)
{
    config_ = config;
}

const DegeneracyConfig&
AdaptiveWeightCalculator::
config() const
{
    return config_;
}

void AdaptiveWeightCalculator::
setMaximumTranslation(
        double w)
{
    config_.translation_max = w;
}

void AdaptiveWeightCalculator::
setRotationWeight(
        double w)
{
    config_.rotation_weight = w;
}

void AdaptiveWeightCalculator::
setGain(
        double k)
{
    config_.weight_gain = k;
}

WeightResult
AdaptiveWeightCalculator::
calculate(
        double score) const
{
    WeightResult result;

    /*
     * 防止异常score导致权重出现非正常数值。
     */
    if(score < 0.0)
    {
        score = 0.0;
    }

    /*
     * 当前论文中的自适应权重模型：
     *
     * w_t =
     * w_max / (1 + k * score)
     */
    result.translation_weight =
            config_.translation_max /
            (1.0 + config_.weight_gain * score);

    /*
     * 当前阶段旋转权重保持固定。
     *
     * 后续如果论文实验表明旋转方向也需要自适应，
     * 再扩展这里。
     */
    result.rotation_weight =
            config_.rotation_weight;

    return result;
}
