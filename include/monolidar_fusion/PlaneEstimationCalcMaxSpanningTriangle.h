/*
 * PlaneEstimationCalcMaxSpanningTriangle.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace Mono_Lidar {
/*
 * Gets of a list of 3D points the three points which span a triangle with a maximum size
 */
class PlaneEstimationCalcMaxSpanningTriangle {
public:
    PlaneEstimationCalcMaxSpanningTriangle(const bool publishPoints = false);

    /*
     * @param distTreshold The minimum distance (in meters) which 2 mutual points must have
     */
    PlaneEstimationCalcMaxSpanningTriangle(const double distTreshold, const bool publishPoints = false);

    /**
     * Selects three corner points from a point-list considering the edge condition to maximize the area of the spanning
     * triangle
     * @param points [in] list of corner point cantidates
     * @param corner1 [out] corner 1 of the biggest spanning triangle
     * @param corner2 [out] corner 2 of the biggest spanning triangle
     * @param corner3 [out] corner 3 of the biggest spanning triangle
     */
    bool CalculatePlaneCorners(const std::vector<Eigen::Vector3d>& points,
                               Eigen::Vector3d& corner1,
                               Eigen::Vector3d& corner2,
                               Eigen::Vector3d& corner3);

    bool CalculatePlaneCorners(const std::vector<Eigen::Vector3d>& points,
                               Eigen::Vector3d& corner1,
                               Eigen::Vector3d& corner2,
                               Eigen::Vector3d& corner3,
                               std::vector<Eigen::Vector3d>& publishPointList);

private:
    double _distTreshold;
    bool _publishPoints;
};
}
