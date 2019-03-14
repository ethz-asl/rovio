/*
 * NeighborFinderPixel.h
 *
 *  Created on: Feb 6, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>

#include "NeighborFinderBase.h"

namespace Mono_Lidar {
class NeighborFinderPixel : public NeighborFinderBase {
public:
    NeighborFinderPixel(const int imgWitdh,
                        const int imgHeight,
                        const int pixelSearchWidth,
                        const int pixelSearchHeight);

    void Initialize(std::shared_ptr<DepthEstimatorParameters>& parameters) override;

    void InitializeLidarProjection(const Eigen::Matrix2Xd& lidarPoints_img_cs,
                                   const Eigen::Matrix3Xd& points_cs_camera,
                                   const std::vector<int>& pointIndex);

    // gets the neighbors of a given feature point including the point itself
    void getNeighbors(const Eigen::Vector2d& featurePoint_image_cs,
                      const Eigen::Matrix3Xd& points_cs_camera,
                      const std::vector<int>& pointIndex,
                      std::vector<int>& pcIndicesCut,
                      const std::shared_ptr<DepthCalcStatsSinglePoint>& calcStats = NULL,
                      const float scaleWidth = 1,
                      const float scaleHeight = 1) override;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // Constants
    const int POINT_NOT_DEFINED = -1;

    Eigen::MatrixXi _img_points_lidar; // Matrix with dim (imgWitdh*imgHeight). Each pixels stores a index of a
                                       // projected (visible) lidar point or -1 if no point aviable at this pixel

    int _imgWitdth;
    int _imgHeight;

    int _pixelSearchWidth;
    int _pixelSearchHeight;
};
}
