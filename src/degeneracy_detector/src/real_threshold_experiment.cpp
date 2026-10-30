#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <iomanip>

struct ExperimentData
{
    double score;
};

int main(int argc, char** argv)
{
    std::string filename = "/home/sj/scan_experiment.csv";

    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Failed to open: "
                  << filename << std::endl;

        return 1;
    }

    std::string line;

    // 跳过表头
    std::getline(file, line);

    std::vector<ExperimentData> data;

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::stringstream ss(line);

        std::string field;
        std::vector<std::string> fields;

        while (std::getline(ss, field, ','))
        {
            fields.push_back(field);
        }

        /*
         * scan_experiment.csv:
         *
         * 1 frame
         * 2 time
         * 3 point_num
         * 4 lambda_max
         * 5 lambda_min
         * 6 score
         * 7 translation_weight
         * 8 rotation_weight
         * 9 direction_x
         * 10 direction_y
         * 11 degenerate
         */

        if (fields.size() < 11)
            continue;

        ExperimentData item;

        // score 是第6列，对应 fields[5]
        item.score = std::stod(fields[5]);

        data.push_back(item);
    }

    file.close();

    std::cout << std::endl;
    std::cout << "========================================"
              << std::endl;
    std::cout << "Real Bag Threshold Experiment"
              << std::endl;
    std::cout << "========================================"
              << std::endl;

    std::cout << "CSV: "
              << filename
              << std::endl;

    std::cout << "Total frames: "
              << data.size()
              << std::endl;

    std::cout << std::endl;

    std::vector<double> thresholds =
    {
        1.5,
        2.0,
        2.5,
        3.0,
        3.5,
        4.0,
        4.2,
        4.4,
        4.5,
        4.6,
        4.64,
        4.7,
        5.0,
        10.0,
        100.0
    };

    std::cout
        << std::left
        << std::setw(12)
        << "Threshold"
        << std::setw(15)
        << "Degenerate"
        << std::setw(15)
        << "Rate(%)"
        << std::endl;

    std::cout
        << "----------------------------------------"
        << std::endl;

    for (double threshold : thresholds)
    {
        size_t count = 0;

        for (const auto& item : data)
        {
            if (item.score > threshold)
            {
                count++;
            }
        }

        double rate = 0.0;

        if (!data.empty())
        {
            rate =
                100.0 *
                static_cast<double>(count) /
                static_cast<double>(data.size());
        }

        std::cout
            << std::left
            << std::setw(12)
            << std::fixed
            << std::setprecision(2)
            << threshold
            << std::setw(15)
            << count
            << std::setw(15)
            << rate
            << std::endl;
    }

    std::cout << std::endl;

    return 0;
}
