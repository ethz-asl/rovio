/*
 * LinePlaneIntersectionOrthogonalTreshold.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include "LinePlaneIntersectionBase.h"

namespace Mono_Lidar {
class LinePlaneIntersectionOrthogonalTreshold : public LinePlaneIntersectionBase {
public:
    LinePlaneIntersectionOrthogonalTreshold(const double treshold);

    bool GetIntersection(const Eigen::Hyperplane<double, 3>& plane,
                         const Eigen::Vector3d& n0,
                         const Eigen::Vector3d& n1,
                         Eigen::Vector3d& intersectionPoint,
                         double& intersectionDistance) override;

    bool GetIntersectionDistance(const Eigen::Vector3d& p1,
                               const Eigen::Vector3d& p2,
                               const Eigen::Vector3d& p3,
                               const Eigen::Vector3d& rayOrigin,
                               const Eigen::Vector3d& rayDirection,
                               double& intersectionDistance) override;

private:
    bool CheckPlaneViewRayOrthogonal(const Eigen::Vector3d& planeNormalVec,
                                     const Eigen::Vector3d& viewingRay,
                                     double treshold);

    double _treshold;
};
}
