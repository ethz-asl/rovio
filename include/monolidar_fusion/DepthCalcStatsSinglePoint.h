/*
 * DepthCalcStatsSinglePoint.h
 *
 *  Created on: Jan 18, 2017
 *      Author: wilczynski
 */

#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include "eDepthResultType.h"


namespace Mono_Lidar {
struct DepthCalcStatsSinglePoint {
    // result type of the depth calculation
    DepthResultType _calcResult;
    // Lidar neigbors which are near the feature point
    std::vector<std::pair<float, float>> _neighbors2d;
    std::vector<std::tuple<float, float, float>> _neighbors3d;

    // Lidar points, which are in the depth bin
    std::vector<std::pair<float, float>> _pointsSegmented2d;
    std::vector<std::tuple<float, float, float>> _pointsSegmented3d;

    // searchrect for neighbor search
    std::pair<int, int> _searchRectTopLeft;
    std::pair<int, int> _searchRectBottomRight;

    // List of bags from histogram
    // std::vector<std::shared_ptr<DepthCalcHistBag>> _bagList;

    // Histogram debugging
    int _histBinCount;
    double _histBinWitdh;
    int _histMinDist;
    int _histMaxDist;
    double _histLowerBorder;
    double _histHigherBorder;
    std::vector<float> _histDepthEntryCount;
    int _histFound;

    // PCA
    std::string _pcaResult;
    Eigen::Vector4d _pcaEigenVector1;
    Eigen::Vector4d _pcaEigenVector2;
    Eigen::Vector4d _pcaEigenVector3;
    Eigen::Vector3d _pcaPlaneAnchor;
    Eigen::Vector3d _pcaPlaneNormal;

    // 2d image feature point coordinates
    int _featureX;
    int _featureY;

    // calculated depth of the feature point
    double _featureDepth;
    Eigen::Vector2d _pointInterpolated2d;
    Eigen::Vector3d _pointInterpolated3d;
};
}
