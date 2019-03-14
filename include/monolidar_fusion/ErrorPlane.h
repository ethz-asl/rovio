/*
 * ErrorPlane.h
 *
 *  Created on: Mar 17, 2017
 *      Author: wilczynski
 */

#pragma once

#include <vector>
#include <Eigen/Eigen>

#include <ceres/ceres.h>

namespace Mono_Lidar {
struct ErrorPlane {
    ErrorPlane(const Eigen::Vector3d& point, const double errorScale) : _point(point), _errorScale(errorScale) {
    }

    template <typename T>
    bool operator()(const T* const planeparams, T* residuals) const {
        // planeparams[0,1,2] are the plane normal planeparams[4] is the distance scalar
        T norm;
        norm = T(
            sqrt(planeparams[0] * planeparams[0] + planeparams[1] * planeparams[1] + planeparams[2] * planeparams[2]));

        // The error is the disance from the given point to the plane
        residuals[0] = T(_errorScale) * (planeparams[0] * T(_point.x()) + planeparams[1] * T(_point.y()) +
                                         planeparams[2] * T(_point.z()) / norm);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& point, const double errorScale = 1) {
        return (new ceres::AutoDiffCostFunction<ErrorPlane, 1, 4>(new ErrorPlane(point, errorScale)));
    }

    Eigen::Vector3d _point;
    double _errorScale;
};
}
