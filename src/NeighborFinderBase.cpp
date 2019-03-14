/*
 * NeighborFinderBase.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/NeighborFinderBase.h"

namespace Mono_Lidar {
void NeighborFinderBase::Initialize(std::shared_ptr<DepthEstimatorParameters>& parameters) {
    _parameters = parameters;
}

void NeighborFinderBase::getNeighbors(const Eigen::Matrix3Xd& points_cs_camera,
                                      const std::vector<int>& pointIndices,
                                      const std::vector<int>& pcIndicesCut,
                                      std::vector<Eigen::Vector3d>& neighbors) {
    using namespace std;

    for (const int index : pcIndicesCut) {
        int indexRaw = pointIndices.at(index);

        neighbors.push_back(Eigen::Vector3d(
            points_cs_camera(0, indexRaw), points_cs_camera(1, indexRaw), points_cs_camera(2, indexRaw)));
    }
}

void NeighborFinderBase::getNeighbors(const Eigen::Matrix3Xd& points_cs_camera,
                                      const std::vector<int>& pcIndicesFull,
                                      std::vector<Eigen::Vector3d>& neighbors) {
    using namespace std;

    for (const int index : pcIndicesFull) {
        neighbors.push_back(
            Eigen::Vector3d(points_cs_camera(0, index), points_cs_camera(1, index), points_cs_camera(2, index)));
    }
}
}
