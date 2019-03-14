/*
 * NeighborFinderPixel.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/NeighborFinderPixel.h"
#include "monolidar_fusion/Logger.h"

namespace Mono_Lidar {

NeighborFinderPixel::NeighborFinderPixel(const int imgWitdh,
                                         const int imgHeight,
                                         const int pixelSearchWidth,
                                         const int pixelSearchHeight)
        : _imgWitdth(imgWitdh), _imgHeight(imgHeight), _pixelSearchWidth(pixelSearchWidth),
          _pixelSearchHeight(pixelSearchHeight), _img_points_lidar(imgWitdh, imgHeight) {
}

void NeighborFinderPixel::Initialize(std::shared_ptr<DepthEstimatorParameters>& parameters) {
    NeighborFinderBase::Initialize(parameters);

    _pixelSearchWidth = _parameters->pixelarea_search_witdh;
    _pixelSearchHeight = _parameters->pixelarea_search_height;
}

void NeighborFinderPixel::InitializeLidarProjection(const Eigen::Matrix2Xd& lidarPoints_img_cs,
                                                    const Eigen::Matrix3Xd& points_cs_camera,
                                                    const std::vector<int>& pointIndex) {
    Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::TransformLidarPointsToImageFrame");

    int doubleProjectionCount = 0;
    using namespace std;

    int pointCount = lidarPoints_img_cs.cols();
    _img_points_lidar.setConstant(POINT_NOT_DEFINED);

    for (int i = 0; i < pointCount; i++) {
        int x_img = lidarPoints_img_cs(0, i);
        int y_img = lidarPoints_img_cs(1, i);

        int indexRaw = pointIndex[i];

        Eigen::Vector3d lidarPoint3D(
            points_cs_camera(0, indexRaw), points_cs_camera(1, indexRaw), points_cs_camera(2, indexRaw));

        // Check if there are multiple lidar point projections at the same image cooridnate and that the point is in
        // front of the camera
        if ((_img_points_lidar(x_img, y_img) == POINT_NOT_DEFINED) && (lidarPoint3D.z() > 0)) {
            // set the index to a lidar point to a image pixel
            _img_points_lidar(x_img, y_img) = i;
        }
    }

    Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::TransformLidarPointsToImageFrame");
}

void NeighborFinderPixel::getNeighbors(const Eigen::Vector2d& featurePoint_image_cs,
                                       const Eigen::Matrix3Xd& points_cs_camera,
                                       const std::vector<int>& pointIndex,
                                       std::vector<int>& pcIndicesCut,
                                       const std::shared_ptr<DepthCalcStatsSinglePoint>& calcStats,
                                       const float scaleWidth,
                                       const float scaleHeight) {
    double halfSizeX = static_cast<double>(_pixelSearchWidth) * 0.5 * static_cast<double>(scaleWidth);
    double halfSizeY = static_cast<double>(_pixelSearchHeight) * 0.5 * static_cast<double>(scaleHeight);

    double leftEdgeX = std::max(featurePoint_image_cs[0] - halfSizeX, 0.);
    double rightEdgeX = std::min(featurePoint_image_cs[0] + halfSizeX, static_cast<double>(_imgWitdth - 1));
    double topEdgeY = std::max(featurePoint_image_cs[1] - halfSizeY, 0.);
    double bottomEdgeY = std::min(featurePoint_image_cs[1] + halfSizeY, static_cast<double>(_imgHeight - 1));

    for (int i = static_cast<int>(topEdgeY); i <= static_cast<int>(bottomEdgeY); i++) {
        for (int j = static_cast<int>(leftEdgeX); j <= static_cast<int>(rightEdgeX); j++) {
            if (_img_points_lidar(j, i) != POINT_NOT_DEFINED) {
                // get the index from the visible (in camera img) point cloud
                int index = _img_points_lidar(j, i);

                if (index == POINT_NOT_DEFINED)
                    continue;

                // get the index of the cut point cloud
                pcIndicesCut.push_back(index);
            }
        }
    }

    // debug log neighbors finder
    if (calcStats != NULL) {
        calcStats->_searchRectTopLeft = std::pair<int, int>((int)leftEdgeX, (int)topEdgeY);
        calcStats->_searchRectBottomRight = std::pair<int, int>((int)rightEdgeX, (int)bottomEdgeY);
    }
}
}
