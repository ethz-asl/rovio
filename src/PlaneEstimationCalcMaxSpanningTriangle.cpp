/*
 * PlaneEstimationCalcMaxSpanningTriangle.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/PlaneEstimationCalcMaxSpanningTriangle.h"

namespace Mono_Lidar {
PlaneEstimationCalcMaxSpanningTriangle::PlaneEstimationCalcMaxSpanningTriangle(const bool publishPoints)
        : _distTreshold(0), _publishPoints(publishPoints) {
}

PlaneEstimationCalcMaxSpanningTriangle::PlaneEstimationCalcMaxSpanningTriangle(const double distTreshold,
                                                                               const bool publishPoints)
        : _distTreshold(distTreshold), _publishPoints(publishPoints) {
}

bool PlaneEstimationCalcMaxSpanningTriangle::CalculatePlaneCorners(const std::vector<Eigen::Vector3d>& points,
                                                                   Eigen::Vector3d& corner1,
                                                                   Eigen::Vector3d& corner2,
                                                                   Eigen::Vector3d& corner3,
                                                                   std::vector<Eigen::Vector3d>& publishPointList) {
    bool result = CalculatePlaneCorners(points, corner1, corner2, corner3);

//#pragma omp critical
    {
        publishPointList.push_back(corner1);
        publishPointList.push_back(corner2);
        publishPointList.push_back(corner3);
    }

    return result;
}

bool PlaneEstimationCalcMaxSpanningTriangle::CalculatePlaneCorners(const std::vector<Eigen::Vector3d>& points,
                                                                   Eigen::Vector3d& corner1,
                                                                   Eigen::Vector3d& corner2,
                                                                   Eigen::Vector3d& corner3) {
    // precheck, needs at least 3 points
    int pointsCount = points.size();

    if (pointsCount < 3) {
        return false;
    }

    // Calc the index of the two points with the greatest distance
    int maxDist_i = -1;
    int maxDist_j = -1;
    double maxdist = -1;
    for (int i = 0; i < pointsCount - 1; i++) {
        for (int j = i + 1; j < pointsCount; j++) {
            double dist = (points[i] - points[j]).squaredNorm();

            if (dist > maxdist) {
                maxdist = dist;
                maxDist_i = i;
                maxDist_j = j;
            }
        }
    }

    // Check if all points lie on the same spot
    if (maxdist <= _distTreshold)
        return false;

    // calculate the third point which sum of distances (to the thirst and second point) is the biggest
    double maxdist2 = -1;
    double maxDist_k = -1;
    for (int k = 0; k < pointsCount - 1; k++) {
        if (k == maxDist_i || k == maxDist_j)
            continue;

        double dist1 = (points[k] - points[maxDist_i]).squaredNorm();

        if (dist1 <= _distTreshold)
            continue;

        double dist2 = (points[k] - points[maxDist_j]).squaredNorm();

        if (dist2 <= _distTreshold)
            continue;

        double dist = dist1 + dist2;

        if (dist > maxdist2) {
            maxdist2 = dist;
            maxDist_k = k;
        }
    }

    if ((maxDist_i == -1) || (maxDist_j == -1) || (maxDist_k == -1))
        return false;

    // assign points
    corner1 = points[maxDist_i];
    corner2 = points[maxDist_j];
    corner3 = points[maxDist_k];


    // neue Methode
    //		int index_max_i = -1;
    //		int index_max_j = -1;
    //		int index_max_k = -1;
    //		double max_area = -1;
    //
    //		for (int i = 0; i < pointsCount-2; i++)
    //		{
    //			for (int j = i + 1; j < pointsCount-1; j++)
    //			{
    //				for (int k = j+1; k < pointsCount; k++)
    //				{
    //					auto a = (points[j] - points[i]).norm();
    //					auto b = (points[k] - points[i]).norm();
    //					auto c = (points[k] - points[j]).norm();
    //
    //					double s = 0.5 * (a+b+c);
    //					double A_Squared = (s*(s-a)*(s-b)*(s-c));
    //
    //					if (A_Squared > max_area)
    //					{
    //						max_area = A_Squared;
    //						index_max_i = i;
    //						index_max_j = j;
    //						index_max_k = k;
    //					}
    //				}
    //			}
    //		}
    //
    //		if ((index_max_i == -1) || (index_max_j == -1) || (index_max_k == -1))
    //			return false;
    //
    //		if (max_area <_distTreshold)
    //			return false;
    //
    //		// assign points
    //		corner1 = points[index_max_i];
    //		corner2 = points[index_max_j];
    //		corner3 = points[index_max_k];

    return true;
}
}
