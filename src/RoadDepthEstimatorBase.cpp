/*
 * RoadDepthEstimatorBase.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/RoadDepthEstimatorBase.h"

namespace Mono_Lidar {
RoadDepthEstimatorBase::RoadDepthEstimatorBase() : _tresholdDepthGlobal(NULL), _tresholdDepthLocal(NULL) {
}

void RoadDepthEstimatorBase::EnableTresholdDepthGlobal(const eTresholdDepthMode& mode,
                                                       const double minValue,
                                                       const double maxValue) {
    _tresholdDepthGlobal = std::make_shared<TresholdDepthGlobal>(mode, minValue, maxValue);
}

void RoadDepthEstimatorBase::EnableTresholdDepthGlobal(const std::shared_ptr<TresholdDepthGlobal>& obj) {
    _tresholdDepthGlobal = obj;
}

void RoadDepthEstimatorBase::EnableTresholdDepthLocal(const eTresholdDepthMode& mode,
                                                      const eTresholdToleranceType toleranceType,
                                                      const double toleranceValue) {
    _tresholdDepthLocal = std::make_shared<TresholdDepthLocal>(mode, toleranceType, toleranceValue);
}

void RoadDepthEstimatorBase::EnableTresholdDepthLocal(const std::shared_ptr<TresholdDepthLocal>& obj) {
    _tresholdDepthLocal = obj;
}
}
