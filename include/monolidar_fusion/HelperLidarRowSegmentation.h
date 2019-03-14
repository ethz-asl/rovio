/*
 * HelperLidarRowSegmentation.h
 *
 *  Created on: Jul 12, 2017
 *      Author: wilczynski
 */

#pragma once

#include <map>
#include <vector>
#include <Eigen/Eigen>

#include "PointcloudData.h"

namespace Mono_Lidar {
class HelperLidarRowSegmentation {
private:
    // key: index of a point; value: row of the given point
    std::map<int, int> _mapIndexRow;
    std::map<int, int> _mapIndexColumn;
    // row - column - index of the point in Matrix2d
    std::vector<std::vector<int>> _points;

    bool _isInitialized = false;

    bool getNeighborRowPoint(const PointcloudData& pointCloudData,
                             const Eigen::Vector2d& featurePoint,
                             const int curRow,
                             std::pair<int, int>& neighborRows);

    int getSingleNeighborRowPoint(const PointcloudData& pointCloudData,
                                  const Eigen::Vector2d& featurePoint,
                                  const int curRow);

    /*
     * Gets the nearest point index in a list of points to a feature point.
     * The points are spreaded through a 2D-Field of view
     *
     * @param pointsImageVisible 2D-Matrixc with all visible points on the image
     * @param featurePoint 2D-Point which nearest neighbor is determinded
     * @param pointList List of selected points from pointsImageVisible
     *
     * @return The Index of the nearest point of the given feature point
     */
    int getNearestPoint(const Eigen::Matrix2Xd& pointsImageVisible,
                        const Eigen::Vector2d& featurePoint,
                        const std::vector<int>& imgPointIndices);

    int selectRowIndex(const PointcloudData& points,
                       const double maxDistSeedToSeed,
                       const Eigen::Vector3d& nearestPoint3D,
                       const std::pair<double, double>& neighborRows);


    /*
     * Returns the lidar row index of a given point
     * @param pointIndex Index of the 2D lidar point due to visible points in the image plane
     */
    int getRowFromPoint(int pointIndex) {
        return _mapIndexRow.at(pointIndex);
    }

    /*
     * Returns the lidar column index of a given point
     * @param pointIndex Index of the 2D lidar point due to visible points in the image plane
     */
    int getColumnFromPoint(int pointIndex) {
        return _mapIndexColumn.at(pointIndex);
    }

    double getMaxDist(const double distTreshold,
                      const double maxDistStartValue,
                      const double maxDistGradient,
                      const double seedDistance);

    /*
     * Selects points in a row around a given seed point until a maximum 3D-distance to the seed point is reached or
     * the distance of two neighbor points exceeds a given distance.
     *
     * @param seedPointIndex Index of the 3D-Seedpoint (visible Index)
     * @param maxDistanceNeighbor Maximum distance between to consecutive points
     * @param maxDistance Maximum distance between a point and the seed point
     * @param pointIndices List in which the selected points are stored (the list is not cleared and the new points are
     * added to older points, if the list isn't empty)
     * @param maxPointCount Maximum points for the selected lidar row (-1 if unbounded)
     */
    void getSelectionFromSeed(const PointcloudData& points,
                              const int seedPointIndex,
                              const double maxDistanceNeighbor,
                              const double maxDistance,
                              std::vector<int>& pointIndices,
                              const int maxPointCount = -1);

public:
    HelperLidarRowSegmentation();

    int calculateNeighborPoints(const PointcloudData& points,
                                const Eigen::Vector2d& featurePoint,
                                const double maxDistTreshold,
                                const double maxDistSeedToSeedStart,
                                const double maxDistSeedToSeedGradient,
                                const double maxDistNeighborToSeedStart,
                                const double maxDistNeighborToSeedGradient,
                                const double maxDistNeighborStart,
                                const double maxDistNeighborGradient,
                                const int maxPointCount,
                                std::vector<int>& pointsSegmentedIndex);

    std::vector<std::vector<int>>& PointIndex() {
        return _points;
    }

    /*
     * Creates the needed data structure for further processing.
     * Must be created once for each new point cloud
     *
     * @param pointsImageVisible 2D-Points which are visible in the camera
     */
    void SegmentPoints(const Eigen::Matrix2Xd& pointsImageVisible);
};
}
