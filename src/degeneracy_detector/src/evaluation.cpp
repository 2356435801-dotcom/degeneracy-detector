#include "evaluation.h"

#include <iomanip>

Evaluation::Evaluation()
{

}

Evaluation::~Evaluation()
{
    if(file_.is_open())
    {
        file_.close();
    }
}

bool Evaluation::open(
        const std::string& filename)
{
    file_.open(filename);

    if(!file_.is_open())
    {
        return false;
    }

    file_

	<<"frame,"

	<<"time,"

	<<"point_num,"

	<<"cov_xx,"

	<<"cov_xy,"

	<<"cov_yy,"

	<<"lambda_max,"

	<<"lambda_min,"

	<<"score,"

	<<"translation_weight,"

	<<"rotation_weight,"

	<<"direction_x,"

	<<"direction_y,"

	<<"degenerate"

	<<std::endl;

    return true;
}

void Evaluation::save(
        int frame_id,
        int point_num,
        const ScanFrame& frame,
        const DegeneracyResult& degeneracy,
        const CovarianceResult& covariance,
        const WeightResult& weight)
{
    if(!file_.is_open())
    {
        return;
    }

    file_

	<<frame_id<<","

	<<frame.stamp.toSec()<<","

	<<point_num<<","

	<<covariance.covariance(0,0)<<","

	<<covariance.covariance(0,1)<<","

	<<covariance.covariance(1,1)<<","

	<<degeneracy.lambda_max<<","

	<<degeneracy.lambda_min<<","

	<<degeneracy.score<<","

	<<weight.translation_weight<<","

	<<weight.rotation_weight<<","

	<<degeneracy.direction.x()<<","

	<<degeneracy.direction.y()<<","

	<<degeneracy.degenerate

	<<std::endl;
}
