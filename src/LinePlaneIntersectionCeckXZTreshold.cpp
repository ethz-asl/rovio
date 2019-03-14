/*
 * LinePlaneIntersectionCeckXZTreshold.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/LinePlaneIntersectionCeckXZTreshold.h"
#include <iostream>

namespace Mono_Lidar {
LinePlaneIntersectionCeckXZTreshold::LinePlaneIntersectionCeckXZTreshold(double treshold) : _treshold(treshold) {
}

bool LinePlaneIntersectionCeckXZTreshold::Check(const std::vector<Eigen::Vector3d>& points) {
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();

    // get the minimal and maximum point values in x and z direction
    for (const auto& point : points) {
        if (point.x() < minX)
            minX = point.x();

        if (point.x() > maxX)
            maxX = point.x();

        if (point.z() < minZ)
            minZ = point.z();

        if (point.z() > maxZ)
            maxZ = point.z();
    }

    double sizeX = maxX - minX;
    double sizeZ = maxZ - minZ;
    // a bigger z-x relation is better for our purpose because it reduces the angle error of the estimated plane in
    // camera x axis
    double relation = sizeZ / sizeX;

    bool result = (relation >= _treshold);

    return result;
}

bool LinePlaneIntersectionCeckXZTreshold::Check(const std::vector<Eigen::Vector3d>& points, double treshold) {
    this->_treshold = treshold;
    return Check(points);
}
}
