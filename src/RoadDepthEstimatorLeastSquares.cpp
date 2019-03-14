/*
 * RoadDepthEstimatorLeastSquares.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/RoadDepthEstimatorLeastSquares.h"
#include "monolidar_fusion/LinePlaneIntersectionNormal.h"

namespace Mono_Lidar {

RoadDepthEstimatorLeastSquares::RoadDepthEstimatorLeastSquares() {
    // Init object for intersection calculation between a plane and a line
    _linePlaneIntersection = std::make_shared<LinePlaneIntersectionNormal>();

    // Object for estimation of a plane with a given list of points
    _planeEstimation = std::make_shared<PlaneEstimationLeastSquares>();
}

std::pair<DepthResultType, double> RoadDepthEstimatorLeastSquares::CalculateDepth(
    const Eigen::Vector2d& point_image,
    const std::shared_ptr<CameraPinhole> _camera,
    const std::vector<Eigen::Vector3d>& planePoints,
    Eigen::Vector3d& pointiNtersection) {
    // Estimate the plane through the given pointlist
    Eigen::Vector3d planeNormal;
    double planeDist;

    _planeEstimation->EstimatePlane(planePoints, planeNormal, planeDist);


    // get the ray through camera center and image feature point
    Eigen::Vector3d viewingRaySupportPoint;
    Eigen::Vector3d viewingRayDirection;
    _camera->getViewingRays(point_image, viewingRaySupportPoint, viewingRayDirection);

    if (viewingRayDirection.z() < 0)
        viewingRayDirection *= -1;

    double depth;

    // Caclculate the intersection between camera raytrace and the plane
    _linePlaneIntersection->GetIntersectionPoint(
        planeNormal, planeDist, viewingRayDirection, viewingRaySupportPoint, pointiNtersection, depth);

    // Check for global tresholds
    if (this->_tresholdDepthGlobal != NULL) {
        auto result = this->_tresholdDepthGlobal->CheckInDepth(depth);
        if (result == eTresholdResult::SmallerMin)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthGlobalSmallerMin, -1);
        else if (result == eTresholdResult::GreaterMax)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthGlobalGreaterMax, -1);
    }

    // check for local tresholds
    if (this->_tresholdDepthLocal != NULL) {
        auto result = this->_tresholdDepthLocal->CheckInBounds(planePoints, depth);
        if (result == eTresholdResult::SmallerMin)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthLocalSmallerMin, -1);
        else if (result == eTresholdResult::GreaterMax)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthLocalGreaterMax, -1);
    }

    return std::pair<DepthResultType, double>(DepthResultType::SuccessRoad, depth);
}
}
