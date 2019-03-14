/*
 * TresholdDepth.h
 *
 *  Created on: Mar 16, 2017
 *      Author: wilczynski
 */

#pragma once

#include "eTresholdDepthMode.h"
#include "eTresholdResult.h"

namespace Mono_Lidar {

/*
 * Verification of estimated depth.
 * Checks if the depth lies between two fixed treshold values.
 *
 * if not and mode == Dispose: the depth is set to -1 and the feature point is set as an outlier
 * it not and mode == Adjust: the depth is set to the min/max-Value
 *
 */
class TresholdDepthGlobal {
public:
    TresholdDepthGlobal(const eTresholdDepthMode& mode, const double minValue, const double maxValue);

    /*
     * Checks if the treshold lies inside two tresholds
     *
     * @param depth The depth which is checked
     */
    eTresholdResult CheckInDepth(double& depth);

private:
    eTresholdDepthMode _mode;
    double _minValue;
    double _maxValue;
};
}
