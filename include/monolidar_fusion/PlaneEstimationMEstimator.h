/*
 * PlaneEstimationMEstimator.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <math.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace Mono_Lidar {
class PlaneEstimationMEstimator {
public:
    PlaneEstimationMEstimator();

    /*
     * Estimates a plane with weighted least squares.
     * The weights are reciprocal of the distance from the measured 3 point and a given initial plane
     *
     * @param points Measured 3d lidar points
     * @param planeInit Precalculated plane which is used to downweight outliers in the pointcloud. Points which lie
     * near the plane are weighted stronger
     * @param resultPlane Resulting plane through the pointcloud
     */
    bool EstimatePlane(const std::vector<Eigen::Vector3d>& points,
                       const Eigen::Hyperplane<double, 3>& planeInit,
                       Eigen::Hyperplane<double, 3>& resulPlane);
};
}
