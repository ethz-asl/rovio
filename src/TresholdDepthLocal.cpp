/*
 * TresholdDepthLocal.cpp
 *
 *  Created on: Mar 17, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/TresholdDepthLocal.h"
#include <string>

namespace Mono_Lidar {
TresholdDepthLocal::TresholdDepthLocal(const eTresholdDepthMode& mode,
                                       const eTresholdToleranceType toleranceType,
                                       const double toleranceValue)
        : _mode(mode), _toleranceType(toleranceType), _toleranceValue(toleranceValue) {
}

eTresholdResult TresholdDepthLocal::CheckInBounds(const std::vector<Eigen::Vector3d>& pointCloud, double& depth) {
    // calculate the min and max depth interval borders of he pointcloud
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();

    for (const auto& point : pointCloud) {
        if (point.z() < minZ)
            minZ = point.z();

        if (point.z() > maxZ)
            maxZ = point.z();
    }

    // Calculate the valid interval
    double depthInterval = maxZ - minZ;
    double borderLow, borderHigh;

    if (_toleranceType == eTresholdToleranceType::relative) {
        double intervalResized = depthInterval * _toleranceValue;
        borderLow = minZ - intervalResized;
        borderHigh = maxZ + intervalResized;
    } else if (_toleranceType == eTresholdToleranceType::absolute) {
        borderLow = minZ - _toleranceValue;
        borderHigh = maxZ + _toleranceValue;
    } else {
        throw "Undefined treshold tolerance mode for tresholdDepthLocal";
    }

    // check if depth lies inside tolerance
    if (depth < borderLow) {
        if (_mode == eTresholdDepthMode::Dispose) {
            depth = -1;
            return eTresholdResult::SmallerMin;
        } else if (_mode == eTresholdDepthMode::Adjust)
            depth = borderLow;
        else
            throw "Undefined treshold depth mode in config (for min): " + std::to_string(_mode);
    } else if (depth > borderHigh) {
        if (_mode == eTresholdDepthMode::Dispose) {
            depth = -1;
            return eTresholdResult::GreaterMax;
        } else if (_mode == eTresholdDepthMode::Adjust)
            depth = borderHigh;
        else
            throw "Undefined treshold depth mode in config (for max): " + std::to_string(_mode);
    }

    return eTresholdResult::InBounds;
}
}
