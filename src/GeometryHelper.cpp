/*
 * GeometryHelper.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: wilczynski
 */

#include "monolidar_fusion/GeometryHelper.h"

namespace Mono_Lidar {

void GeometryHelper::GetIntersectionPoint(Eigen::Vector3d& p1,
                                          Eigen::Vector3d& p2,
                                          Eigen::Vector3d& p3,
                                          Eigen::Vector3d& n1,
                                          Eigen::Vector3d& n2,
                                          Eigen::Vector3d& intersectionPoint,
                                          bool& isInTriangle) {

    // create a 2d plane between the 3 given points
    Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(p1, p2, p3);

    // create raycast
    Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(n1, n2);

    // calculate intersection between the raycast and the plane
    auto intersectionMatrix = line.intersectionPoint(plane);
    intersectionPoint.x() = intersectionMatrix(0, 0);
    intersectionPoint.y() = intersectionMatrix(1, 0);
    intersectionPoint.z() = intersectionMatrix(2, 0);

    double ntersectDistance = line.intersection(plane);
}
} /* namespace lidorb_ros_tool */
