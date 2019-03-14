/*
 * RoadDepthEstimatorMEstimator.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>

#include "PlaneEstimationMEstimator.h"
#include "RoadDepthEstimatorBase.h"

namespace Mono_Lidar {
class RoadDepthEstimatorMEstimator : public RoadDepthEstimatorBase {
public:
    RoadDepthEstimatorMEstimator();

    /*
     * Calculates the depth of a feature point in an image (with a triangulated point)
     */
    std::pair<DepthResultType, double> CalculateDepth(const Eigen::Vector2d& point_image,
                                                      const std::shared_ptr<CameraPinhole> _camera,
                                                      const std::vector<Eigen::Vector3d>& planePoints,
                                                      Eigen::Vector3d& pointIntersection) override;

    void setPlanePrior(const Eigen::Hyperplane<double, 3>& plane);

private:
    // Object for estimation of a plane with a given list of points
    std::shared_ptr<PlaneEstimationMEstimator> _planeEstimation;

    // Plane prior from ransac groundplane estimation
    bool _isPriorSet;
    Eigen::Hyperplane<double, 3> _priorPlane;
};
}
