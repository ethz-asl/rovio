/*
 * RoadDepthEstimatorMaxSpanningTriangle.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <memory>
#include <Eigen/Eigen>

#include "LinePlaneIntersectionBase.h"
#include "LinePlaneIntersectionCeckXZTreshold.h"
#include "PlaneEstimationCalcMaxSpanningTriangle.h"
#include "RoadDepthEstimatorBase.h"
#include "eDepthResultType.h"

namespace Mono_Lidar {
class RoadDepthEstimatorMaxSpanningTriangle : public RoadDepthEstimatorBase {
public:
    /*
     * @param xzTreshold The minimum relation between the size of a given
     */
    RoadDepthEstimatorMaxSpanningTriangle(const double xzTreshold);

    std::pair<DepthResultType, double> CalculateDepth(const Eigen::Vector2d& point_image,
                                                      const std::shared_ptr<CameraPinhole> _camera,
                                                      const std::vector<Eigen::Vector3d>& planePoints,
                                                      Eigen::Vector3d& pointiNtersection) override;

private:
    // Object for estimation of a plane with a given list of points
    std::shared_ptr<PlaneEstimationCalcMaxSpanningTriangle> _planeEstimation;
    // object for checking the planes planarity so the plane is not a line
    std::shared_ptr<LinePlaneIntersectionCeckXZTreshold> _planarityChecker;
};
}
