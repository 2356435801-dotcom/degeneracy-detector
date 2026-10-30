#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "data_type.h"
#include "covariance.h"
#include "degeneracy_analyzer.h"
#include "degeneracy_config.h"

std::vector<Eigen::Vector2d>
CreateDegeneratePointCloud()
{
    std::vector<Eigen::Vector2d> points;

    for(int x = -100; x <= 100; ++x)
    {
        double xx = static_cast<double>(x) * 0.1;

        points.emplace_back(xx, 0.01);
        points.emplace_back(xx, -0.01);
    }

    return points;
}

std::vector<Eigen::Vector2d>
Rotate(
    const std::vector<Eigen::Vector2d>& points,
    double angle)
{
    Eigen::Rotation2Dd rotation(angle);

    std::vector<Eigen::Vector2d> result;

    for(const auto& p : points)
    {
        result.push_back(rotation * p);
    }

    return result;
}

void Run(
    double angle_deg)
{
    double angle =
        angle_deg * M_PI / 180.0;

    auto points =
        Rotate(
            CreateDegeneratePointCloud(),
            angle);

    ScanFrame frame;

    frame.points = points;

    CovarianceCalculator calculator;

    CovarianceResult covariance_result;

    if(!calculator.compute(
            frame,
            covariance_result))
    {
        std::cout
            << "Covariance failed."
            << std::endl;

        return;
    }

    DegeneracyConfig config;

    DegeneracyAnalyzer analyzer(config);

    DegeneracyResult result;

    if(!analyzer.analyze(
            covariance_result,
            result))
    {
        std::cout
            << "Analysis failed."
            << std::endl;

        return;
    }

    double theta =
        std::atan2(
            result.direction.y(),
            result.direction.x());

    std::cout
        << "Rotation = "
        << angle_deg
        << " deg"
        << std::endl;

    std::cout
        << "Score    = "
        << result.score
        << std::endl;

    std::cout
        << "Direction = "
        << result.direction.transpose()
        << std::endl;

    std::cout
        << "Theta    = "
        << theta * 180.0 / M_PI
        << " deg"
        << std::endl;

    std::cout
        << "Degenerate = "
        << result.degenerate
        << std::endl;

    std::cout
        << "--------------------------------"
        << std::endl;
}

int main()
{
    std::cout
        << "========================================"
        << std::endl;

    std::cout
        << "Rotation Invariance Experiment"
        << std::endl;

    std::cout
        << "========================================"
        << std::endl;

    Run(0);
    Run(15);
    Run(30);
    Run(45);
    Run(60);
    Run(90);
    Run(135);

    return 0;
}
