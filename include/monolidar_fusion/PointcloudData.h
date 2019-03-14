/*
 * PointcloudData.h
 *
 *  Created on: Jul 18, 2017
 *      Author: wilczynski
 */

#pragma once

#include <math.h>
#include <Eigen/Eigen>

namespace Mono_Lidar {
class PointcloudData {

private:
public:
    Eigen::Matrix3Xd _points_cs_lidar;  // lidar points in lidar-coordinates
    Eigen::Matrix3Xd _points_cs_camera; // lidar points in camera-coordinates
    Eigen::Array<bool, 1, Eigen::Dynamic>
        _pointsInImgRange;             // lookUp-Map to check if a pointcloud point is visible in the image frame.
    Eigen::Matrix2Xd _points_cs_image; // lidar points in image coordinates
    Eigen::Matrix2Xd
        _points_cs_image_visible; // subset of lidar points which are visible in the image in image coordimates
    // list of indizes of points which are visible in the image (used for projection from the cut pointcloud
    // (points from pointcloud which are visible in the image frame) into the original pointcloud)
    std::vector<int> _pointIndex;

    PointcloudData() {
    }

    static double CalcDistanceSquared(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
        return (pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2) + pow(vec1.z() - vec2.z(), 2));
    }

    static double CalcDistanceSquared(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
        return (pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2));
    }

    static double CalcDistance(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
        return sqrt(pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2) + pow(vec1.z() - vec2.z(), 2));
    }

    static double CalcDistance(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
        return sqrt(pow(vec1.x() - vec2.x(), 2) + pow(vec1.y() - vec2.y(), 2));
    }


    int FromCutToOriginalIndex(int cutIndex) const {
        return _pointIndex.at(cutIndex);
    }

    Eigen::Vector2d get2DPointFromCut(int cutIndex) const {
        return Eigen::Vector2d(_points_cs_image_visible(0, cutIndex), _points_cs_image_visible(1, cutIndex));
    }

    Eigen::Vector3d get3DPointFromOriginal(int originalIndex) const {
        return Eigen::Vector3d(_points_cs_camera(0, originalIndex),
                               _points_cs_camera(1, originalIndex),
                               _points_cs_camera(2, originalIndex));
    }

    Eigen::Vector3d get3DPointFromCut(int cutIndex) const {
        return get3DPointFromOriginal(FromCutToOriginalIndex(cutIndex));
    }
};
}
