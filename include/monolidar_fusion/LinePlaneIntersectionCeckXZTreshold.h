/*
 * LinePlaneIntersectionCeckXZTreshold.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace Mono_Lidar {
class LinePlaneIntersectionCeckXZTreshold {
public:
    LinePlaneIntersectionCeckXZTreshold(double treshold);

    // points are in camera coordinates
    bool Check(const std::vector<Eigen::Vector3d>& points);

    // points are in camera coordinates
    bool Check(const std::vector<Eigen::Vector3d>& points, double treshold);

private:
    double _treshold;
};
}
