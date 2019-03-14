/*
 * RoadDepthEstimatorMaxSpanningTriangle.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/RoadDepthEstimatorMaxSpanningTriangle.h"
#include "monolidar_fusion/LinePlaneIntersectionNormal.h"

namespace Mono_Lidar {

RoadDepthEstimatorMaxSpanningTriangle::RoadDepthEstimatorMaxSpanningTriangle(const double xzTreshold) {
    // Init object for intersection calculation between a plane and a line
    _linePlaneIntersection = std::make_shared<LinePlaneIntersectionNormal>();

    // Object for estimation of a plane with a given list of points
    _planeEstimation = std::make_shared<PlaneEstimationCalcMaxSpanningTriangle>();

    // object for checking the planes planarity so the plane is not a line
    _planarityChecker = std::make_shared<LinePlaneIntersectionCeckXZTreshold>(xzTreshold);
}

std::pair<DepthResultType, double> RoadDepthEstimatorMaxSpanningTriangle::CalculateDepth(
    const Eigen::Vector2d& point_image,
    const std::shared_ptr<CameraPinhole> _camera,
    const std::vector<Eigen::Vector3d>& planePoints,
    Eigen::Vector3d& pointiNtersection) {
    // Get the spanning triangle from lidar points
    Eigen::Vector3d corner1;
    Eigen::Vector3d corner2;
    Eigen::Vector3d corner3;

    // Find the 3 corners which describes a plane
    if (!_planeEstimation->CalculatePlaneCorners(planePoints, corner1, corner2, corner3))
        return std::pair<DepthResultType, double>(DepthResultType::RadiusSearchInsufficientPoints, -1);

    // check if the plane is planar
    if (!_planarityChecker->Check(planePoints))
        return std::pair<DepthResultType, double>(DepthResultType::InsufficientRoadPoints, -1);

    // get the ray through camera center and image feature point
    Eigen::Vector3d viewingRaySupportPoint;
    Eigen::Vector3d viewingRayDirection;
    _camera->getViewingRays(point_image, viewingRaySupportPoint, viewingRayDirection);

    if (viewingRayDirection.z() < 0)
        viewingRayDirection *= -1;

    double depth;

    // Caclculate the intersection between camera raytrace and the plane
    _linePlaneIntersection->GetIntersectionPoint(
        corner1, corner2, corner3, viewingRayDirection, viewingRaySupportPoint, pointiNtersection, depth);

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
