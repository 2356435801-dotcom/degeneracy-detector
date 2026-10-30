#ifndef EVALUATION_H
#define EVALUATION_H

#include <fstream>
#include <string>

#include "data_type.h"
#include "adaptive_weight.h"

class Evaluation
{
public:

    Evaluation();

    ~Evaluation();

    bool open(const std::string& filename);

    void save(
            int frame_id,
            int point_num,
            const ScanFrame& frame,
            const DegeneracyResult& degeneracy,
            const CovarianceResult& covariance,
            const WeightResult& weight);

private:

    std::ofstream file_;
};

#endif
