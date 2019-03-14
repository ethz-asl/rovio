/*
 * HelperLidarRowSegmentation.cpp
 *
 *  Created on: Jul 12, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/HelperLidarRowSegmentation.h"
#include <iostream>
#include <limits>

namespace Mono_Lidar {


HelperLidarRowSegmentation::HelperLidarRowSegmentation() {
}

void HelperLidarRowSegmentation::SegmentPoints(const Eigen::Matrix2Xd& pointsImageVisible) {
    using namespace std;
    _points.clear();
    _mapIndexRow.clear();
    _mapIndexColumn.clear();

    int count = pointsImageVisible.cols();

    double lastX = std::numeric_limits<int>::min();

    for (int i = 0; i < count; i++) {
        double curX = pointsImageVisible(0, i);
        if (curX > (lastX + 50)) {
            // create new row
            _points.push_back(std::vector<int>());
        }

        int curRow = _points.size() - 1;
        int curColumn = _points.at(curRow).size();

        _points.at(curRow).push_back(i);
        _mapIndexRow.insert(std::pair<int, int>(i, curRow));
        _mapIndexColumn.insert(std::pair<int, int>(i, curColumn));

        lastX = curX;
    }

    _isInitialized = true;
}

int HelperLidarRowSegmentation::getNearestPoint(const Eigen::Matrix2Xd& pointsImageVisible,
                                                const Eigen::Vector2d& featurePoint,
                                                const std::vector<int>& imgPointIndices) {
    double minDist = std::numeric_limits<double>::max();
    double minIndex = -1;

    for (int index : imgPointIndices) {
        double deltaX = fabs(pointsImageVisible(0, index) - featurePoint.x());
        double deltaY = fabs(pointsImageVisible(1, index) - featurePoint.y());
        double distance = deltaX * deltaX + deltaY * deltaY;

        if (distance < minDist) {
            minDist = distance;
            minIndex = index;
        }
    }

    return minIndex;
}

int HelperLidarRowSegmentation::getSingleNeighborRowPoint(const PointcloudData& pointCloudData,
                                                          const Eigen::Vector2d& featurePoint,
                                                          const int curRow) {
    using namespace std;

    const auto& points = _points.at(curRow);

    Eigen::Vector2d lastPoint(std::numeric_limits<int>::min(), 0);
    Eigen::Vector2d curPoint(0, 0);
    int indexCur = -1;
    int indexLast = -1;
    bool found = false;

    for (int index : points) {
        indexCur = index;
        curPoint = pointCloudData.get2DPointFromCut(index);

        if ((curPoint.x() < featurePoint.x()) && (lastPoint.x() >= featurePoint.x())) {
            found = true;
            break;
        }

        indexLast = indexCur;
        lastPoint = curPoint;
    }

    if (!found)
        return -1;

    if (indexLast == -1)
        return indexCur;

    auto distCur = pow(curPoint.x() - featurePoint.x(), 2) + pow(curPoint.y() - featurePoint.y(), 2);
    auto distLast = pow(curPoint.x() - lastPoint.x(), 2) + pow(curPoint.y() - lastPoint.y(), 2);

    if (distCur < distLast)
        return indexCur;
    else
        return indexLast;
}

bool HelperLidarRowSegmentation::getNeighborRowPoint(const PointcloudData& pointCloudData,
                                                     const Eigen::Vector2d& featurePoint,
                                                     const int curRow,
                                                     std::pair<int, int>& neighborRows) {
    int topRow = curRow - 1;
    int bottomRow = curRow + 1;

    bool topIsValid = topRow >= 0;
    bool bottomIsValid = bottomRow < _points.size();

    int topIndex = -1;
    int bottomIndex = -1;

    if (topIsValid)
        topIndex = getSingleNeighborRowPoint(pointCloudData, featurePoint, topRow);

    if (bottomIsValid)
        bottomIndex = getSingleNeighborRowPoint(pointCloudData, featurePoint, bottomRow);

    if (topIndex == -1)
        topIsValid = false;

    if (bottomIndex == -1)
        bottomIsValid = false;

    // get the second nearest
    if (!topIsValid && !bottomIsValid)
        return false;
    else if (topIsValid && !bottomIsValid) {
        neighborRows = std::make_pair(topIndex, -1);
        return true;
    } else if (!topIsValid && bottomIsValid) {
        neighborRows = std::make_pair(bottomIndex, -1);
        return true;
    }

    // Calculate Distances
    auto pointTop = pointCloudData.get2DPointFromCut(topIndex);
    double distTop = pow(pointTop.x() - featurePoint.x(), 2) + pow(pointTop.y() - featurePoint.y(), 2);
    auto pointBottom = pointCloudData.get2DPointFromCut(bottomIndex);
    double distBottom = pow(pointBottom.x() - featurePoint.x(), 2) + pow(pointBottom.y() - featurePoint.y(), 2);

    if (distTop < distBottom)
        neighborRows = std::make_pair(topIndex, bottomIndex);
    else
        neighborRows = std::make_pair(bottomIndex, topIndex);

    return true;
}

void HelperLidarRowSegmentation::getSelectionFromSeed(const PointcloudData& points,
                                                      const int seedPointIndex,
                                                      const double maxDistanceNeighbor,
                                                      const double maxDistanceSeed,
                                                      std::vector<int>& pointIndices,
                                                      const int maxPointCount) {


    using namespace std;

    // get points to the left
    int rowIndex = getRowFromPoint(seedPointIndex);
    int columnIndex = getColumnFromPoint(seedPointIndex);

    int curColumnIndex = columnIndex;
    auto seedPoint3D = points.get3DPointFromCut(seedPointIndex);
    auto lastPoint3D = seedPoint3D;

    auto seedPoint2D = points.get2DPointFromCut(seedPointIndex);

    if (maxPointCount < 0) {
        while (true) {
            curColumnIndex++;

            if (curColumnIndex >= _points.at(rowIndex).size())
                break;

            int pointIndex = _points.at(rowIndex).at(curColumnIndex);
            auto point3D = points.get3DPointFromCut(pointIndex);
            auto point2D = points.get2DPointFromCut(pointIndex);
            auto distanceSeed = PointcloudData::CalcDistance(point3D, seedPoint3D);

            if (distanceSeed > maxDistanceSeed)
                break;

            auto distanceNeighbor = PointcloudData::CalcDistance(point3D, lastPoint3D);

            if (distanceNeighbor > maxDistanceNeighbor)
                break;

            lastPoint3D = point3D;

            pointIndices.push_back(pointIndex);
        }

        // get points to the right
        curColumnIndex = columnIndex;

        while (true) {
            curColumnIndex--;

            if (curColumnIndex < 0)
                break;

            int pointIndex = _points.at(rowIndex).at(curColumnIndex);
            auto point2D = points.get2DPointFromCut(pointIndex);
            auto point3D = points.get3DPointFromCut(pointIndex);
            auto distanceSeed = PointcloudData::CalcDistance(point3D, seedPoint3D);

            if (distanceSeed > maxDistanceSeed)
                break;

            auto distanceNeighbor = PointcloudData::CalcDistance(point3D, lastPoint3D);

            if (distanceNeighbor > maxDistanceNeighbor)
                break;

            lastPoint3D = point3D;

            pointIndices.push_back(pointIndex);
        }
    } else {

        int curColumnIndexPos = columnIndex + 1;
        int curColumnIndexNeg = columnIndex - 1;

        while (true) {
            // Check if slected index is in range
            if (curColumnIndexPos >= _points.at(rowIndex).size())
                break;

            if (curColumnIndexNeg < 0)
                break;

            // point in positive direction
            int pointIndexPos = _points.at(rowIndex).at(curColumnIndexPos);
            auto point3DPos = points.get3DPointFromCut(pointIndexPos);
            auto distanceSeedPos = PointcloudData::CalcDistance(point3DPos, seedPoint3D);

            // point in negative direction
            int pointIndexNeg = _points.at(rowIndex).at(curColumnIndexNeg);
            auto point3DNeg = points.get3DPointFromCut(pointIndexNeg);
            auto distanceSeedNeg = PointcloudData::CalcDistance(point3DNeg, seedPoint3D);

            double distanceSeed;
            int pointIndex;

            if (distanceSeedPos <= distanceSeedNeg) {
                pointIndex = pointIndexPos;
                distanceSeed = distanceSeedPos;
                curColumnIndexPos++;
            } else {
                pointIndex = pointIndexNeg;
                distanceSeed = distanceSeedNeg;
                curColumnIndexNeg--;
            }

            if (distanceSeed > maxDistanceSeed)
                break;

            pointIndices.push_back(pointIndex);

            if (pointIndices.size() >= maxPointCount)
                break;
        }
    }
}

int HelperLidarRowSegmentation::selectRowIndex(const PointcloudData& points,
                                               const double maxDistSeedToSeed,
                                               const Eigen::Vector3d& nearestPoint3D,
                                               const std::pair<double, double>& neighborRows) {
    Eigen::Vector3d nearestPointSecondRow3d = points.get3DPointFromCut(neighborRows.first);

    // Check if the distance between the two seedpoints doesn't exceed a given treshold
    auto seedDistance = PointcloudData::CalcDistance(nearestPoint3D, nearestPointSecondRow3d);

    if (seedDistance <= maxDistSeedToSeed)
        return neighborRows.first;

    // first row is out of range, check second row
    if (neighborRows.second == -1)
        return -1;

    nearestPointSecondRow3d = points.get3DPointFromCut(neighborRows.second);
    seedDistance = PointcloudData::CalcDistance(nearestPoint3D, nearestPointSecondRow3d);

    if (seedDistance <= maxDistSeedToSeed)
        return neighborRows.second;

    return -1;
}

double HelperLidarRowSegmentation::getMaxDist(const double distTreshold,
                                              const double maxDistStartValue,
                                              const double maxDistGradient,
                                              const double seedDistance) {
    if (seedDistance <= distTreshold)
        return maxDistStartValue;

    double delta = seedDistance - maxDistStartValue;
    double distance = maxDistStartValue + delta * maxDistGradient;

    return distance;
}

int HelperLidarRowSegmentation::calculateNeighborPoints(const PointcloudData& points,
                                                        const Eigen::Vector2d& featurePoint,
                                                        const double maxDistTreshold,
                                                        const double maxDistSeedToSeedStart,
                                                        const double maxDistSeedToSeedGradient,
                                                        const double maxDistNeighborToSeedStart,
                                                        const double maxDistNeighborToSeedGradient,
                                                        const double maxDistNeighborStart,
                                                        const double maxDistNeighborGradient,
                                                        const int maxPointCount,
                                                        std::vector<int>& pointsSegmentedIndex) {
    if (!_isInitialized)
        throw "'HelperLidarRowSegmentation' not initialized. Call 'SegmentPoints' first.";

    // Get nearest Lidar-Point to the feature point
    const int nearestPointIndex = getNearestPoint(points._points_cs_image_visible, featurePoint, pointsSegmentedIndex);

    if (nearestPointIndex == -1)
        return -4;

    int row1 = getRowFromPoint(nearestPointIndex);

    Eigen::Vector3d nearestPoint3D = points.get3DPointFromCut(nearestPointIndex);
    // Get a point to the next nearest row to the feature point
    std::pair<int, int> neighborRows;
    if (!getNeighborRowPoint(points, featurePoint, row1, neighborRows))
        return -1;

    // Got a seed point of two rows
    // Now use region growing on each row using the given seed points
    pointsSegmentedIndex.clear();

    const double maxDistSeedToSeed =
        getMaxDist(maxDistTreshold, maxDistSeedToSeedStart, maxDistSeedToSeedGradient, nearestPoint3D.z());
    const auto secondRowIndex = selectRowIndex(points, maxDistSeedToSeed, nearestPoint3D, neighborRows);

    if (secondRowIndex < 0)
        return -2;

    const double maxDistNeighborToSeed =
        getMaxDist(maxDistTreshold, maxDistNeighborToSeedStart, maxDistNeighborToSeedGradient, nearestPoint3D.z());
    const double maxDistNeighbor =
        getMaxDist(maxDistTreshold, maxDistNeighborStart, maxDistNeighborGradient, nearestPoint3D.z());

    if (maxPointCount == -1)
        getSelectionFromSeed(
            points, nearestPointIndex, maxDistNeighborToSeed, maxDistNeighbor, pointsSegmentedIndex, maxPointCount);
    else
        getSelectionFromSeed(
            points, nearestPointIndex, maxDistNeighborToSeed, maxDistNeighbor, pointsSegmentedIndex, maxPointCount / 2);

    int sizeBefore = pointsSegmentedIndex.size();
    getSelectionFromSeed(
        points, secondRowIndex, maxDistNeighborToSeed, maxDistNeighbor, pointsSegmentedIndex, maxPointCount);
    int sizeAfter = pointsSegmentedIndex.size();

    if (sizeBefore == sizeAfter)
        return -3;

    return 1;
}
}
