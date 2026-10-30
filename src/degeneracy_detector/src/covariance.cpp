#include "covariance.h"

#include <Eigen/Eigenvalues>

bool CovarianceCalculator::compute(
        const ScanFrame& frame,
        CovarianceResult& result) const
{
    // 至少需要两个点
    if(frame.points.size() < 2)
    {
        return false;
    }

    //----------------------------------------------------------------------
    // Step 1 : 计算质心
    //----------------------------------------------------------------------

    result.centroid.setZero();

    for(const auto& point : frame.points)
    {
        result.centroid += point;
    }

    result.centroid /= static_cast<double>(frame.points.size());

    //----------------------------------------------------------------------
    // Step 2 : 计算协方差矩阵
    //----------------------------------------------------------------------

    result.covariance.setZero();

    for(const auto& point : frame.points)
    {
        Eigen::Vector2d diff = point - result.centroid;

        result.covariance += diff * diff.transpose();
    }

    result.covariance /=
            static_cast<double>(frame.points.size() - 1);

    //----------------------------------------------------------------------
    // Step 3 : PCA（特征值分解）
    //----------------------------------------------------------------------

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(
            result.covariance);

    if(solver.info() != Eigen::Success)
    {
        return false;
    }

    result.eigen_values =
            solver.eigenvalues();

    result.eigen_vectors =
            solver.eigenvectors();

    return true;
}
