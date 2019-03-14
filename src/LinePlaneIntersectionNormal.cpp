/*
 * LinePlaneIntersectionNormal.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/LinePlaneIntersectionNormal.h"

namespace Mono_Lidar {
bool LinePlaneIntersectionNormal::GetIntersection(const Eigen::Hyperplane<double, 3>& plane,
                                                  const Eigen::Vector3d& n0,
                                                  const Eigen::Vector3d& n1,
                                                  Eigen::Vector3d& intersectionPoint,
                                                  double& intersectionDistance) {
    // create raycast
    Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(n0, n1);

    // calculate intersection between the raycast and the plane

    intersectionPoint = line.intersectionPoint(plane);

    // intersectionDistance = line.intersection(plane);
    intersectionDistance = (n0 - intersectionPoint).norm();

    return true;
}

bool LinePlaneIntersectionNormal::GetIntersectionDistance(const Eigen::Vector3d& p1,
                                                                      const Eigen::Vector3d& p2,
                                                                      const Eigen::Vector3d& p3,
                                                                      const Eigen::Vector3d& rayOrigin,
                                                                      const Eigen::Vector3d& rayDirection,
                                                                      double& intersectionDistance){
    Eigen::Vector3d v0(p3-p1);
    Eigen::Vector3d v1(p2-p1);
    Eigen::Vector3d planeNormal = v0.cross(v1);

    Eigen::Vector3d centroid = (p1 + p2 + p3) / 3.0;
    intersectionDistance = (rayOrigin - centroid).norm();
    return true;
}

}
