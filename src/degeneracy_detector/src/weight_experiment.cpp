#include <iostream>
#include <iomanip>
#include <vector>

#include "adaptive_weight.h"
#include "degeneracy_config.h"


int main(int argc, char** argv)
{
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Adaptive Weight Experiment" << std::endl;
    std::cout << "========================================" << std::endl;

    // ========================================================
    // 使用当前算法配置
    // ========================================================

    DegeneracyConfig config;

    AdaptiveWeightCalculator calculator(config);


    std::cout << std::endl;

    std::cout << "Configuration:" << std::endl;

    std::cout << "translation_max = "
              << config.translation_max
              << std::endl;

    std::cout << "weight_gain      = "
              << config.weight_gain
              << std::endl;

    std::cout << "rotation_weight  = "
              << config.rotation_weight
              << std::endl;


    // ========================================================
    // 测试Score
    // ========================================================

    std::vector<double> scores =
    {
        1.0,
        1.5,
        2.0,
        2.5,
        3.0,
        3.5,
        4.0,
        4.5,
        5.0,
        6.0,
        8.0,
        10.0,
        15.0,
        20.0,
        30.0,
        50.0,
        100.0,
        1000.0
    };


    // ========================================================
    // 输出结果
    // ========================================================

    std::cout << std::endl;

    std::cout
        << std::left
        << std::setw(12) << "Score"
        << std::setw(20) << "TranslationWeight"
        << std::setw(18) << "RotationWeight"
        << std::endl;

    std::cout
        << "--------------------------------------------------"
        << std::endl;


    for(double score : scores)
    {
        WeightResult result =
            calculator.calculate(score);

        std::cout
            << std::fixed
            << std::setprecision(6)
            << std::left
            << std::setw(12) << score
            << std::setw(20)
                << result.translation_weight
            << std::setw(18)
                << result.rotation_weight
            << std::endl;
    }


    std::cout << std::endl;

    return 0;
}
