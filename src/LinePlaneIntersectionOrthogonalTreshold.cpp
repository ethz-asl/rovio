/*
 * LinePlaneIntersectionOrthogonalTreshold.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/LinePlaneIntersectionOrthogonalTreshold.h"

namespace Mono_Lidar {

LinePlaneIntersectionOrthogonalTreshold::LinePlaneIntersectionOrthogonalTreshold(const double treshold)
        : _treshold(treshold) {
}

bool LinePlaneIntersectionOrthogonalTreshold::GetIntersection(const Eigen::Hyperplane<double, 3>& plane,
                                                              const Eigen::Vector3d& n0,
                                                              const Eigen::Vector3d& n1,
                                                              Eigen::Vector3d& intersectionPoint,
                                                              double& intersectionDistance) {
    // create raycast
    Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(n0, n1);

    // Check if the treshold condition is fulfilled
    Eigen::Vector3d lineNormal = n1.normalized();
    Eigen::Vector3d planeNormal = plane.normal();

    if (!CheckPlaneViewRayOrthogonal(planeNormal.normalized(), lineNormal, _treshold))
        return false;

    // calculate intersection between the raycast and the plane
    intersectionPoint = line.intersectionPoint(plane);

    intersectionDistance = (n0 - intersectionPoint).norm();

    return true;
}

bool LinePlaneIntersectionOrthogonalTreshold::GetIntersectionDistance(const Eigen::Vector3d& p1,
                                                                      const Eigen::Vector3d& p2,
                                                                      const Eigen::Vector3d& p3,
                                                                      const Eigen::Vector3d& rayOrigin,
                                                                      const Eigen::Vector3d& rayDirection,
                                                                      double& intersectionDistance){
  Eigen::Vector3d v0(p3-p1);
  Eigen::Vector3d v1(p2-p1);
  Eigen::Vector3d planeNormal = v0.cross(v1).normalized();

  if (!CheckPlaneViewRayOrthogonal(planeNormal, rayDirection, _treshold))
    return false;

  Eigen::Vector3d centroid = (p1 + p2 + p3) / 3.0;
  intersectionDistance = (rayOrigin - centroid).norm();
  return true;
}

bool LinePlaneIntersectionOrthogonalTreshold::CheckPlaneViewRayOrthogonal(const Eigen::Vector3d& planeNormalVec,
                                                                          const Eigen::Vector3d& viewingRay,
                                                                          double treshold) {
    return (fabs(planeNormalVec.dot(viewingRay)) >= treshold);
}
}
