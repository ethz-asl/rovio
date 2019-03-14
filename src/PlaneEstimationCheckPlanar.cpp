/*
 * PlaneEstimationCheckPlanar.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/PlaneEstimationCheckPlanar.h"

namespace Mono_Lidar {

PlaneEstimationCheckPlanar::PlaneEstimationCheckPlanar() : _treshold(0) {
}

PlaneEstimationCheckPlanar::PlaneEstimationCheckPlanar(const double treshold) : _treshold(treshold) {
}

bool PlaneEstimationCheckPlanar::CheckPlanar(const Eigen::Vector3d& corner1,
                                             const Eigen::Vector3d& corner2,
                                             const Eigen::Vector3d& corner3) {
    // calc edges
    Eigen::Vector3d edge1 = corner2 - corner1;
    Eigen::Vector3d edge2 = corner3 - corner1;
    Eigen::Vector3d edge3 = corner3 - corner2;
    edge1.normalize();
    edge2.normalize();
    edge3.normalize();

    // calc length of the cross product of all edges
    Eigen::Vector3d cross12 = edge1.cross(edge2);
    Eigen::Vector3d cross13 = edge1.cross(edge3);
    Eigen::Vector3d cross23 = edge2.cross(edge3);

    double length12 = cross12.norm();
    double length13 = cross13.norm();
    double length23 = cross23.norm();

    // Check planar condition (TRESHOLD is normalized: between 0 and 1)
    bool check12 = (length12 >= _treshold);
    bool check13 = (length13 >= _treshold);
    bool check23 = (length23 >= _treshold);

    return (check12 && check13 && check23);
}
}
