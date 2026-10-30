#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>

#include "data_type.h"
#include "covariance.h"


// ============================================================
// 创建正常环境
// ============================================================

std::vector<Eigen::Vector2d> CreateNormalEnvironment()
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
// 创建退化环境
// ============================================================

std::vector<Eigen::Vector2d> CreateDegenerateEnvironment()
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

void AddGaussianNoise(
    std::vector<Eigen::Vector2d>& points,
    double sigma,
    std::mt19937& generator)
{
    std::normal_distribution<double> distribution(0.0, sigma);

    for (auto& point : points)
    {
        point.x() += distribution(generator);
        point.y() += distribution(generator);
    }
}


// ============================================================
// 计算 Score
// ============================================================

double CalculateScore(
    const std::vector<Eigen::Vector2d>& points)
{
    ScanFrame frame;

    frame.points = points;

    CovarianceCalculator calculator;

    CovarianceResult result;

    if (!calculator.compute(frame, result))
    {
        return -1.0;
    }

    double lambda_min = result.eigen_values(0);
    double lambda_max = result.eigen_values(1);

    if (lambda_min <= 0.0)
    {
        return 1e12;
    }

    return lambda_max / lambda_min;
}


// ============================================================
// main
// ============================================================

int main()
{
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Threshold Sensitivity Experiment" << std::endl;
    std::cout << "========================================" << std::endl;

    const int trials = 30;

    const std::vector<double> sigmas =
    {
        0.0,
        0.1,
        0.2,
        0.5,
        1.0
    };

    const std::vector<double> thresholds =
    {
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
        80.0,
        100.0,
        150.0,
        200.0,
        500.0
    };


    std::cout << std::endl;
    std::cout << "Detection rate of degenerate environment"
              << std::endl;

    std::cout << std::endl;

    std::cout << "Threshold";

    for (double sigma : sigmas)
    {
        std::cout << "\tSigma=" << sigma;
    }

    std::cout << std::endl;

    std::cout << "------------------------------------------------------------"
              << std::endl;


    std::random_device rd;

    std::mt19937 generator(rd());


    for (double threshold : thresholds)
    {
        std::cout << threshold;

        for (double sigma : sigmas)
        {
            int detected = 0;

            for (int trial = 0;
                 trial < trials;
                 ++trial)
            {
                std::vector<Eigen::Vector2d> points =
                    CreateDegenerateEnvironment();

                AddGaussianNoise(
                    points,
                    sigma,
                    generator);

                double score =
                    CalculateScore(points);

                if (score > threshold)
                {
                    detected++;
                }
            }

            double detection_rate =
                100.0 *
                static_cast<double>(detected) /
                trials;

            std::cout << "\t"
                      << detection_rate
                      << "%";
        }

        std::cout << std::endl;
    }


    // ========================================================
    // 正常环境误检
    // ========================================================

    std::cout << std::endl;
    std::cout << "False positive rate of normal environment"
              << std::endl;

    std::cout << std::endl;

    std::cout << "Threshold";

    for (double sigma : sigmas)
    {
        std::cout << "\tSigma=" << sigma;
    }

    std::cout << std::endl;

    std::cout << "------------------------------------------------------------"
              << std::endl;


    for (double threshold : thresholds)
    {
        std::cout << threshold;

        for (double sigma : sigmas)
        {
            int false_positive = 0;

            for (int trial = 0;
                 trial < trials;
                 ++trial)
            {
                std::vector<Eigen::Vector2d> points =
                    CreateNormalEnvironment();

                AddGaussianNoise(
                    points,
                    sigma,
                    generator);

                double score =
                    CalculateScore(points);

                if (score > threshold)
                {
                    false_positive++;
                }
            }

            double false_positive_rate =
                100.0 *
                static_cast<double>(false_positive) /
                trials;

            std::cout << "\t"
                      << false_positive_rate
                      << "%";
        }

        std::cout << std::endl;
    }


    std::cout << std::endl;
    std::cout << "Experiment finished."
              << std::endl;

    return 0;
}
