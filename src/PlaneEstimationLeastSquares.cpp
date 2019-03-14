/*
 * PlaneEstimationLeastSquares.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/PlaneEstimationLeastSquares.h"
#include "monolidar_fusion/ErrorPlane.h"

namespace Mono_Lidar {
PlaneEstimationLeastSquares::PlaneEstimationLeastSquares() {
}

bool PlaneEstimationLeastSquares::EstimatePlane(const std::vector<Eigen::Vector3d>& points,
                                                Eigen::Vector3d& planeNormal,
                                                double& resultDistance) {
    // set initial pose
    double ext[] = {0, 0, 0, 0};

    ceres::Problem problem;

    for (auto const& point : points) {
        ceres::CostFunction* cost_function = ErrorPlane::Create(point);
        problem.AddResidualBlock(cost_function, NULL /* squared loss */, ext);
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // write output
    planeNormal = Eigen::Vector3d(ext[0], ext[1], ext[2]);
    resultDistance = ext[4];

    return true;
}
}
