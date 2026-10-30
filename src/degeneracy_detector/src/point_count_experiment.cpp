#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

#include "data_type.h"
#include "covariance.h"
#include "degeneracy_analyzer.h"
#include "degeneracy_config.h"

std::vector<Eigen::Vector2d> CreateEnvironment(int point_count)
{
    std::vector<Eigen::Vector2d> points;

    std::mt19937 gen(42);

    std::normal_distribution<double> noise_y(0.0, 0.02);

    for(int i = 0; i < point_count; ++i)
    {
        double x =
            -10.0 +
            20.0 * static_cast<double>(i)
            / static_cast<double>(point_count - 1);

        double y = noise_y(gen);

        points.emplace_back(x, y);
    }

    return points;
}

int main()
{
    std::cout << std::endl;
    std::cout << "========================================"
              << std::endl;
    std::cout << "Point Count Robustness Experiment"
              << std::endl;
    std::cout << "========================================"
              << std::endl;

    std::vector<int> counts =
    {
        20,
        50,
        100,
        200,
        500,
        1000,
        2000,
        5000,
        10000
    };

    DegeneracyConfig config;
    DegeneracyAnalyzer analyzer(config);

    for(int count : counts)
    {
        ScanFrame frame;

        frame.points = CreateEnvironment(count);

        CovarianceCalculator calculator;

        CovarianceResult covariance;

        if(!calculator.compute(frame, covariance))
        {
            std::cout
                << "Point count "
                << count
                << " covariance FAILED"
                << std::endl;

            continue;
        }

        DegeneracyResult result;

        if(!analyzer.analyze(
                covariance,
                result))
        {
            std::cout
                << "Point count "
                << count
                << " analysis FAILED"
                << std::endl;

            continue;
        }

        std::cout
            << "Points = "
            << count
            << "  Score = "
            << result.score
            << "  Degenerate = "
            << (result.degenerate ? "YES" : "NO")
            << std::endl;
    }

    return 0;
}
