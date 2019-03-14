/*
 * TresholdDepthLocal.h
 *
 *  Created on: Mar 17, 2017
 *      Author: wilczynski
 */

#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include "eTresholdDepthMode.h"
#include "eTresholdResult.h"

namespace Mono_Lidar {
enum eTresholdToleranceType {
    absolute = 0,
    relative = 1,
};

/*
 * Verification of estimated depth.
 * Checks if the depth lies between two fixed treshold values.
 * The treshold values are calculated using the lowest and highest z (depth) value of a list of lidar 3D points.
 * Afterwards an additional tolerance for the bounds is added.
 *
 * @param mode
 * if depth lies outside and mode == Dispose: the depth is set to -1 and the feature point is set as an outlier
 * it depth lies outside and mode == Adjust: the depth is set to the min/max-Value
 *
 * @param toleranceType
 * Adds an additional tolerance to the given bounds.
 * toleranceType == absolute: The tolerance value is added as an absolute value to the bounds
 * toleranceType == relatuive: The added tolerance is a relative value of the spanned depth (max - min depth of the 3d
 * points)
 *
 */
class TresholdDepthLocal {
public:
    TresholdDepthLocal(const eTresholdDepthMode& mode,
                       const eTresholdToleranceType toleranceType,
                       const double toleranceValue);
    // nico todo: don't ever let this function be called, it is HORRIBLY SLOW
    eTresholdResult CheckInBounds(const std::vector<Eigen::Vector3d>& pointCloud, double& depth);

private:
    eTresholdDepthMode _mode;
    eTresholdToleranceType _toleranceType;
    double _toleranceValue;
};
}
