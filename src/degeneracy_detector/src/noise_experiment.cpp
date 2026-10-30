#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include <Eigen/Dense>

#include "data_type.h"
#include "covariance.h"
#include "degeneracy_analyzer.h"
#include "degeneracy_config.h"


// ============================================================
// 生成正常环境
// ============================================================

std::vector<Eigen::Vector2d>
CreateNormalEnvironment()
{
    std::vector<Eigen::Vector2d> points;

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

std::vector<Eigen::Vector2d>
CreateDegenerateEnvironment()
{
    std::vector<Eigen::Vector2d> points;

    for (int x = -100; x <= 100; ++x)
    {
        double xx = static_cast<double>(x) * 0.1;

        points.emplace_back(xx, 0.01);
        points.emplace_back(xx, -0.01);
    }

    return points;
}


// ============================================================
// 添加高斯噪声
// ============================================================

std::vector<Eigen::Vector2d>
AddGaussianNoise(
    const std::vector<Eigen::Vector2d>& input,
    double sigma,
    std::mt19937& generator)
{
    std::normal_distribution<double> noise(0.0, sigma);

    std::vector<Eigen::Vector2d> output;

    output.reserve(input.size());

    for (const auto& point : input)
    {
        Eigen::Vector2d noisy_point;

        noisy_point.x() =
            point.x() + noise(generator);

        noisy_point.y() =
            point.y() + noise(generator);

        output.push_back(noisy_point);
    }

    return output;
}


// ============================================================
// 单次分析
// ============================================================

bool AnalyzePointCloud(
    const std::vector<Eigen::Vector2d>& points,
    double& score,
    bool& degenerate)
{
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
        return false;
    }

    DegeneracyConfig config;

    DegeneracyAnalyzer analyzer(config);

    DegeneracyResult result;

    bool analyze_ok =
        analyzer.analyze(
            covariance_result,
            result);

    if (!analyze_ok)
    {
        return false;
    }

    score = result.score;

    degenerate = result.degenerate;

    return true;
}


// ============================================================
// 运行一个噪声水平
// ============================================================

void RunNoiseExperiment(
    const std::string& environment_name,
    const std::vector<Eigen::Vector2d>& original_points,
    double sigma,
    int trials,
    std::mt19937& generator,
    std::ofstream& csv)
{
    std::vector<double> scores;

    int detected_degenerate = 0;

    for (int i = 0; i < trials; ++i)
    {
        std::vector<Eigen::Vector2d> noisy_points =
            AddGaussianNoise(
                original_points,
                sigma,
                generator);

        double score = 0.0;

        bool degenerate = false;

        bool ok =
            AnalyzePointCloud(
                noisy_points,
                score,
                degenerate);

        if (!ok)
        {
            std::cerr
                << "Analysis failed."
                << std::endl;

            continue;
        }

        scores.push_back(score);

        if (degenerate)
        {
            detected_degenerate++;
        }
    }


    if (scores.empty())
    {
        return;
    }


    // --------------------------------------------------------
    // 统计
    // --------------------------------------------------------

    double sum = 0.0;

    for (double value : scores)
    {
        sum += value;
    }

    double mean =
        sum / scores.size();


    double sum_squared = 0.0;

    for (double value : scores)
    {
        sum_squared +=
            (value - mean) *
            (value - mean);
    }

    double std =
        std::sqrt(
            sum_squared /
            scores.size());


    double min_score =
        *std::min_element(
            scores.begin(),
            scores.end());

    double max_score =
        *std::max_element(
            scores.begin(),
            scores.end());


    double detection_rate =
        100.0 *
        detected_degenerate /
        scores.size();


    // --------------------------------------------------------
    // 输出
    // --------------------------------------------------------

    std::cout
        << std::fixed
        << std::setprecision(6);

    std::cout
        << std::left
        << std::setw(12)
        << sigma
        << std::setw(15)
        << mean
        << std::setw(15)
        << std
        << std::setw(15)
        << min_score
        << std::setw(15)
        << max_score
        << std::setw(15)
        << detection_rate
        << "%"
        << std::endl;


    // --------------------------------------------------------
    // CSV
    // --------------------------------------------------------

    csv
        << environment_name
        << ","
        << sigma
        << ","
        << mean
        << ","
        << std
        << ","
        << min_score
        << ","
        << max_score
        << ","
        << detected_degenerate
        << ","
        << scores.size()
        << ","
        << detection_rate
        << "\n";
}


// ============================================================
// main
// ============================================================

int main()
{
    std::cout << std::endl;
    std::cout
        << "========================================"
        << std::endl;

    std::cout
        << "Noise Robustness Experiment"
        << std::endl;

    std::cout
        << "========================================"
        << std::endl;


    // --------------------------------------------------------
    // 实验参数
    // --------------------------------------------------------

    const int trials = 30;

    const std::vector<double> noise_levels =
    {
        0.0,
        0.005,
        0.01,
        0.02,
        0.05,
        0.10,
        0.20,
        0.50,
        1.00
    };


    // 固定随机种子
    // 保证实验可以重复
    std::mt19937 generator(12345);


    // --------------------------------------------------------
    // 创建点云
    // --------------------------------------------------------

    std::vector<Eigen::Vector2d>
        normal_environment =
        CreateNormalEnvironment();

    std::vector<Eigen::Vector2d>
        degenerate_environment =
        CreateDegenerateEnvironment();


    // --------------------------------------------------------
    // CSV
    // --------------------------------------------------------

    std::ofstream csv(
        "/home/sj/noise_experiment.csv");

    csv
        << "environment,"
        << "sigma,"
        << "mean_score,"
        << "std_score,"
        << "min_score,"
        << "max_score,"
        << "detected_degenerate,"
        << "total_trials,"
        << "detection_rate"
        << "\n";


    // --------------------------------------------------------
    // 表头
    // --------------------------------------------------------

    std::cout
        << std::left
        << std::setw(12)
        << "Sigma"
        << std::setw(15)
        << "MeanScore"
        << std::setw(15)
        << "StdScore"
        << std::setw(15)
        << "MinScore"
        << std::setw(15)
        << "MaxScore"
        << std::setw(15)
        << "Detection"
        << std::endl;

    std::cout
        << "------------------------------------------------------------"
        << std::endl;


    // ========================================================
    // 正常环境
    // ========================================================

    std::cout << std::endl;

    std::cout
        << "Normal Environment"
        << std::endl;

    std::cout
        << "------------------------------------------------------------"
        << std::endl;

    for (double sigma : noise_levels)
    {
        RunNoiseExperiment(
            "normal",
            normal_environment,
            sigma,
            trials,
            generator,
            csv);
    }


    // ========================================================
    // 退化环境
    // ========================================================

    std::cout << std::endl;

    std::cout
        << "Degenerate Environment"
        << std::endl;

    std::cout
        << "------------------------------------------------------------"
        << std::endl;

    for (double sigma : noise_levels)
    {
        RunNoiseExperiment(
            "degenerate",
            degenerate_environment,
            sigma,
            trials,
            generator,
            csv);
    }


    csv.close();


    std::cout << std::endl;

    std::cout
        << "Experiment finished."
        << std::endl;

    std::cout
        << "CSV:"
        << std::endl;

    std::cout
        << "/home/sj/noise_experiment.csv"
        << std::endl;


    return 0;
}
