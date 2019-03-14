/*
 * PlaneEstimationCheckPlanar.h
 *
 *  Created on: Mar 16, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>

namespace Mono_Lidar {
/*
 * Checks if a triangle described by three 3D-Points is planar
 */
class PlaneEstimationCheckPlanar {
public:
    PlaneEstimationCheckPlanar();
    PlaneEstimationCheckPlanar(const double treshold);

    /**
     * 	Checks if a triangle from given 3 edge points is planar
     *  It's not planar if the points lie on the same coordinates or on a line
     *  @param corner1 [in] First corner of the spanning triangle
     *  @param corner2 [in] Second corner of the spanning triangle
     *  @param corner3 [in] Third corner of the spanning triangle
     *  @return True if the angle between all edges is not too small
     */
    bool CheckPlanar(const Eigen::Vector3d& corner1, const Eigen::Vector3d& corner2, const Eigen::Vector3d& corner3);

private:
    double _treshold;
};
}
