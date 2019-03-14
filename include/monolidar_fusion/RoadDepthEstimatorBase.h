/*
 * RoadDepthEstimatorBase.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include "camera_pinhole.h"

#include "LinePlaneIntersectionBase.h"
#include "TresholdDepthGlobal.h"
#include "TresholdDepthLocal.h"
#include "eDepthResultType.h"

namespace Mono_Lidar {
class RoadDepthEstimatorBase {
public:
    virtual ~RoadDepthEstimatorBase() {
    }

    RoadDepthEstimatorBase();

    virtual std::pair<DepthResultType, double> CalculateDepth(const Eigen::Vector2d& point_image,
                                                              const std::shared_ptr<CameraPinhole> _camera,
                                                              const std::vector<Eigen::Vector3d>& planePoints,
                                                              Eigen::Vector3d& pointiNtersection) = 0;

    void EnableTresholdDepthGlobal(const eTresholdDepthMode& mode, const double minValue, const double maxValue);

    void EnableTresholdDepthGlobal(const std::shared_ptr<TresholdDepthGlobal>& obj);

    void EnableTresholdDepthLocal(const eTresholdDepthMode& mode,
                                  const eTresholdToleranceType toleranceType,
                                  const double toleranceValue);

    void EnableTresholdDepthLocal(const std::shared_ptr<TresholdDepthLocal>& obj);

protected:
    std::shared_ptr<LinePlaneIntersectionBase> _linePlaneIntersection;
    std::shared_ptr<TresholdDepthGlobal> _tresholdDepthGlobal;
    std::shared_ptr<TresholdDepthLocal> _tresholdDepthLocal;
};
}
