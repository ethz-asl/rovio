/*
 * RoadDepthEstimatorLeastSquares.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>

#include "PlaneEstimationLeastSquares.h"
#include "RoadDepthEstimatorBase.h"

namespace Mono_Lidar {
class RoadDepthEstimatorLeastSquares : public RoadDepthEstimatorBase {
public:
    RoadDepthEstimatorLeastSquares();

    std::pair<DepthResultType, double> CalculateDepth(const Eigen::Vector2d& point_image,
                                                      const std::shared_ptr<CameraPinhole> _camera,
                                                      const std::vector<Eigen::Vector3d>& planePoints,
                                                      Eigen::Vector3d& pointiNtersection) override;

private:
    // Object for estimation of a plane with a given list of points
    std::shared_ptr<PlaneEstimationLeastSquares> _planeEstimation;
};
}
