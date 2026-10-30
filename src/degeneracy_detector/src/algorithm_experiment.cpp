#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "data_type.h"
#include "covariance.h"
#include "degeneracy_analyzer.h"
#include "degeneracy_config.h"
#include "adaptive_weight.h"
#include "degeneracy_engine.h"


// ============================================================
// 生成正常环境
// ============================================================

std::vector<Eigen::Vector2d> CreateNormalEnvironment()
{
    std::vector<Eigen::Vector2d> points;

    // 近似方形环境
    for (int x = -5; x <= 5; ++x)
    {
        for (int y = -5; y <= 5; ++y)
        {
            points.emplace_back(
                static_cast<double>(x),
                static_cast<double>(y));
        }
    }

    return points;
}


// ============================================================
// 生成退化环境
// ============================================================

std::vector<Eigen::Vector2d> CreateDegenerateEnvironment()
{
    std::vector<Eigen::Vector2d> points;

    // 长条形点云
    for (int x = -100; x <= 100; ++x)
    {
        double xx = static_cast<double>(x) * 0.1;

        points.emplace_back(xx, 0.01);
        points.emplace_back(xx, -0.01);
    }

    return points;
}


// ============================================================
// 执行一次实验
// ============================================================

void RunExperiment(
    const std::string& name,
    const std::vector<Eigen::Vector2d>& points)
{
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Experiment: " << name << std::endl;
    std::cout << "Point count: " << points.size() << std::endl;
    std::cout << "========================================" << std::endl;

    ScanFrame frame;

    frame.points = points;

    CovarianceCalculator covariance_calculator;

    CovarianceResult covariance_result;

    bool covariance_ok =
        covariance_calculator.compute(
            frame,
            covariance_result);

    if (!covariance_ok)
    {
        std::cout << "Covariance calculation FAILED."
                  << std::endl;

        return;
    }

    std::cout << "Covariance matrix:" << std::endl;
    std::cout << covariance_result.covariance
              << std::endl;

    std::cout << std::endl;

    std::cout << "Eigenvalues:" << std::endl;
    std::cout << covariance_result.eigen_values
              << std::endl;

    double lambda_min =
        covariance_result.eigen_values(0);

    double lambda_max =
        covariance_result.eigen_values(1);

    double score =
        lambda_max / lambda_min;

    std::cout << std::endl;

    std::cout << "Lambda min : "
              << lambda_min
              << std::endl;

    std::cout << "Lambda max : "
              << lambda_max
              << std::endl;

    std::cout << "Score      : "
              << score
              << std::endl;


    // --------------------------------------------------------
    // 退化分析
    // --------------------------------------------------------

    DegeneracyConfig config;

    DegeneracyAnalyzer analyzer(config);

    DegeneracyResult degeneracy_result;

    bool analyze_ok =
        analyzer.analyze(
            covariance_result,
            degeneracy_result);

    if (!analyze_ok)
    {
        std::cout << "Degeneracy analysis FAILED."
                  << std::endl;

        return;
    }

    std::cout << "Threshold  : "
              << config.degeneracy_threshold
              << std::endl;

    std::cout << "Degenerate : "
              << (degeneracy_result.degenerate
                      ? "TRUE"
                      : "FALSE")
              << std::endl;

    std::cout << "Direction  : "
              << degeneracy_result.direction.transpose()
              << std::endl;


    // --------------------------------------------------------
    // 自适应权重
    // --------------------------------------------------------

    AdaptiveWeightCalculator weight_calculator(config);

    WeightResult weight_result =
        weight_calculator.calculate(
            degeneracy_result.score);

    std::cout << std::endl;

    std::cout << "Translation weight : "
              << weight_result.translation_weight
              << std::endl;

    std::cout << "Rotation weight    : "
              << weight_result.rotation_weight
              << std::endl;
}


// ============================================================
// main
// ============================================================

int main(int argc, char** argv)
{
    std::cout << std::endl;
    std::cout << "========================================"
              << std::endl;

    std::cout << "Degeneracy Algorithm Experiment"
              << std::endl;

    std::cout << "========================================"
              << std::endl;


    // 正常环境
    RunExperiment(
        "Normal Environment",
        CreateNormalEnvironment());


    // 退化环境
    RunExperiment(
        "Degenerate Environment",
        CreateDegenerateEnvironment());


    return 0;
}
