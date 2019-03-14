/*
 * GeometryHelper.h
 *
 *  Created on: Dec 6, 2016
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>

namespace Mono_Lidar {

class GeometryHelper {
public:
    void GetIntersectionPoint(Eigen::Vector3d& p1,
                              Eigen::Vector3d& p2,
                              Eigen::Vector3d& p3,
                              Eigen::Vector3d& n1,
                              Eigen::Vector3d& n2,
                              Eigen::Vector3d& intersectionPoint,
                              bool& isInTriangle);
};

} /* namespace lidorb_ros_tool */
