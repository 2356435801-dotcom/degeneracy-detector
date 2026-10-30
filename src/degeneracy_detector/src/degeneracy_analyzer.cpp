#include "degeneracy_analyzer.h"

#include <cmath>

DegeneracyAnalyzer::
DegeneracyAnalyzer()
    : config_()
{
}

DegeneracyAnalyzer::
DegeneracyAnalyzer(
        const DegeneracyConfig& config)
    : config_(config)
{
}

void DegeneracyAnalyzer::
setConfig(
        const DegeneracyConfig& config)
{
    config_ = config;
}

const DegeneracyConfig&
DegeneracyAnalyzer::
config() const
{
    return config_;
}

void DegeneracyAnalyzer::
setThreshold(
        double threshold)
{
    config_.degeneracy_threshold =
            threshold;
}

double DegeneracyAnalyzer::
threshold() const
{
    return config_.degeneracy_threshold;
}

bool DegeneracyAnalyzer::
analyze(
        const CovarianceResult& covariance,
        DegeneracyResult& result) const
{
    /*
     * Eigen::SelfAdjointEigenSolver
     * 返回的特征值按升序排列：
     *
     * eigen_values(0) = lambda_min
     * eigen_values(1) = lambda_max
     */
    const double lambda_min =
            covariance.eigen_values(0);

    const double lambda_max =
            covariance.eigen_values(1);

    /*
     * 防止最小特征值接近0。
     */
    if(lambda_min < config_.minimum_eigenvalue)
    {
        return false;
    }

    /*
     * 保存特征值。
     */
    result.lambda_min =
            lambda_min;

    result.lambda_max =
            lambda_max;

    /*
     * 退化指标：
     *
     * score = lambda_max / lambda_min
     */
    result.score =
            lambda_max /
            lambda_min;

    /*
     * 最大特征值对应的特征向量
     * 是点云的主方向。
     *
     * 对于长走廊：
     *
     * 主方向通常对应走廊的延伸方向。
     */
    result.direction =
            covariance.eigen_vectors.col(1);

    /*
     * 根据阈值判断是否退化。
     */
    result.degenerate =
            (result.score >
             config_.degeneracy_threshold);

    return true;
}
