/*
 * PlaneEstimationMEstimator.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "monolidar_fusion/ErrorPlane.h"
#include "monolidar_fusion/PlaneEstimationMEstimator.h"

namespace Mono_Lidar {
PlaneEstimationMEstimator::PlaneEstimationMEstimator() {
}

bool PlaneEstimationMEstimator::EstimatePlane(const std::vector<Eigen::Vector3d>& points,
                                              const Eigen::Hyperplane<double, 3>& plane,
                                              Eigen::Hyperplane<double, 3>& resulPlane) {
    // copy coordinates to  matrix in Eigen format

    // calculate centroid
    // Eigen::Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // Calculate weighted center
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::VectorXd weights(points.size());
    double weightsSum = 0;

    for (int i = 0; i < points.size(); i++) {
        weights[i] = 1 / plane.absDistance(points[i]);
        center += weights[i] * points.at(i);
        weightsSum += weights[i];
    }

    center /= weightsSum;

    // Solve
    Eigen::Matrix3Xd mat(3, points.size());

    for (int i = 0; i < points.size(); i++) {
        double weightSqrt = sqrt((double)weights[i]);
        mat.col(i) = weightSqrt * (points[i] - center);
    }

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = mat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d planeNormal = svd.matrixU().rightCols<1>();

    resulPlane = Eigen::Hyperplane<double, 3>(planeNormal.normalized(), center);

    return true;
}
}
