/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/crs_matrix.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

bool EvaluateHessian(
    ceres::Problem* const problem, double* const pose,
    const double characteristic_length,
    const std::vector<ceres::ResidualBlockId>& residual_blocks,
    CeresScanMatcher2D::HessianDiagnostics* const output) {
  ceres::Problem::EvaluateOptions evaluate_options;
  evaluate_options.parameter_blocks = {pose};
  evaluate_options.residual_blocks = residual_blocks;
  evaluate_options.apply_loss_function = true;
  evaluate_options.num_threads = 1;

  ceres::CRSMatrix sparse_jacobian;
  if (!problem->Evaluate(evaluate_options, nullptr, nullptr, nullptr,
                         &sparse_jacobian)) {
    output->failure_reason = "ceres_evaluate_failed";
    return false;
  }
  if (sparse_jacobian.num_cols != 3 || sparse_jacobian.num_rows <= 0) {
    output->failure_reason = "unexpected_jacobian_dimensions";
    return false;
  }

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(
      sparse_jacobian.num_rows, sparse_jacobian.num_cols);
  for (int row = 0; row < sparse_jacobian.num_rows; ++row) {
    for (int entry = sparse_jacobian.rows[row];
         entry < sparse_jacobian.rows[row + 1]; ++entry) {
      const double value = sparse_jacobian.values[entry];
      if (!std::isfinite(value)) {
        output->failure_reason = "non_finite_jacobian";
        return false;
      }
      jacobian(row, sparse_jacobian.cols[entry]) = value;
    }
  }

  // Express yaw perturbations as equivalent metres at the scan RMS radius.
  jacobian.col(2) /= characteristic_length;
  Eigen::Matrix3d hessian = jacobian.transpose() * jacobian;
  hessian = 0.5 * (hessian + hessian.transpose());
  if (!hessian.allFinite()) {
    output->failure_reason = "non_finite_hessian";
    return false;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(hessian);
  if (solver.info() != Eigen::Success ||
      !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    output->failure_reason = "eigendecomposition_failed";
    return false;
  }
  const double maximum_eigenvalue = solver.eigenvalues()(2);
  const double negative_tolerance =
      1e-12 * std::max(1., std::abs(maximum_eigenvalue));
  if (solver.eigenvalues()(0) < -negative_tolerance) {
    output->failure_reason = "negative_hessian_eigenvalue";
    return false;
  }

  output->num_residuals = sparse_jacobian.num_rows;
  output->num_parameters = sparse_jacobian.num_cols;
  output->eigenvalues = solver.eigenvalues().cwiseMax(0.);
  output->weak_direction = solver.eigenvectors().col(0).normalized();
  const double minimum_eigenvalue = output->eigenvalues(0);
  const double rank_tolerance =
      1e-12 * std::max(1., output->eigenvalues(2));
  output->condition_number =
      minimum_eigenvalue > rank_tolerance
          ? output->eigenvalues(2) / minimum_eigenvalue
          : std::numeric_limits<double>::infinity();
  output->valid = true;
  return true;
}

}  // namespace

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_enable_local_observability_diagnostics(
      parameter_dictionary->HasKey("enable_local_observability_diagnostics") &&
      parameter_dictionary->GetBool("enable_local_observability_diagnostics"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary,
                               LocalObservabilityDiagnostics* const diagnostics)
    const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  ceres::ResidualBlockId measurement_residual_block = nullptr;
  CHECK_GT(options_.occupied_space_weight(), 0.);
  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      measurement_residual_block = problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
    case GridType::TSDF:
      measurement_residual_block = problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D&>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
  }
  CHECK_GT(options_.translation_weight(), 0.);
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.translation_weight(), target_translation),
      nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(options_.rotation_weight(), 0.);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.rotation_weight(), ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  if (diagnostics != nullptr) {
    *diagnostics = LocalObservabilityDiagnostics{};
    diagnostics->scan_match_duration_ms = summary->total_time_in_seconds * 1000.;
    const auto diagnostic_start = std::chrono::steady_clock::now();
    double squared_radius_sum = 0.;
    for (const auto& point : point_cloud) {
      squared_radius_sum += point.position.head<2>().cast<double>().squaredNorm();
    }
    diagnostics->characteristic_length =
        point_cloud.empty()
            ? 0.
            : std::sqrt(squared_radius_sum /
                        static_cast<double>(point_cloud.size()));
    if (!std::isfinite(diagnostics->characteristic_length) ||
        diagnostics->characteristic_length < 1e-3) {
      diagnostics->failure_reason = point_cloud.empty()
                                        ? "empty_point_cloud"
                                        : "invalid_scan_characteristic_length";
    } else {
      const bool measurement_valid = EvaluateHessian(
          &problem, ceres_pose_estimate, diagnostics->characteristic_length,
          {measurement_residual_block}, &diagnostics->measurement);
      const bool objective_valid = EvaluateHessian(
          &problem, ceres_pose_estimate, diagnostics->characteristic_length,
          {}, &diagnostics->objective);
      diagnostics->valid = measurement_valid && objective_valid;
      if (!diagnostics->valid) {
        diagnostics->failure_reason =
            !measurement_valid ? diagnostics->measurement.failure_reason
                               : diagnostics->objective.failure_reason;
      }
    }
    diagnostics->diagnostic_duration_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - diagnostic_start)
            .count();
  }

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
