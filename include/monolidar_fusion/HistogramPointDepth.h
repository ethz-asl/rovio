/*
 * PointHistogram.h
 *
 *  Created on: Dec 22, 2016
 *      Author: wilczynski
 */

#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include "DepthCalcStatsSinglePoint.h"
#include "Histogram.h"

namespace Mono_Lidar {

class PointHistogram {
public:
    /*
     * @brief Segments a lsit of points by it's depth using a histogram and returns the biggest found local bin
     * (maxmimum).
     * @param inputPoints Points that will be segmented by it's depth
     * @param neighborsIndex Indices related to the inputPoints. Must have the same length
     * @param inputDepths Depths of the inputPoints. The points will be segmented by the depths
     * @param binWidth Size of a single bin of the histogram
     * @param minimalMaximumSize The minimum value for a bin to be considered as a local maximum. If the bin value is
     * smaller, it is ignored.
     * @param output List of all 3d Points inside the local maximum bin
     * @param outputIndex List of the indices of all 3d points inside the local maximum bin
     * @param lowerBorder The lower border (depth value) of the first local maximum bin
     * @param higherBorder The higher border (depth value) of the first local maximum bin
     *
     * @return bool True, if a local maximum with the minimum value (point count in the bin) has been found, else false
     */
    static bool FilterPointsMinDistBlob(const std::vector<Eigen::Vector3d>& inputPoints,
                                        const std::vector<int>& neighborsIndex,
                                        const Eigen::VectorXd& inputDepths,
                                        const double binWitdh,
                                        const int minimalMaximumSize,
                                        std::vector<Eigen::Vector3d>& output,
                                        std::vector<int>& outputIndex,
                                        double& lowerBorder,
                                        double& higherBorder,
                                        const std::shared_ptr<DepthCalcStatsSinglePoint>& debugStats = NULL);

    /*
     * Returns the nearest points of the given points. The nearest point is considered as the point with the smallest
     * depth value.
     * @param inputPoints Points that will be segmented by it's depth
     * @param neighborsIndex Indices related to the inputPoints. Must have the same length
     * @param inputDepths Depths of the inputPoints. The points will be segmented by the depths
     * @param output The Single nearest point
     * @param outputIndex The index of the single nearest point
     *
     * @return bool True, if a nearest point has been found
     *
     */
    static bool GetNearestPoint(const std::vector<Eigen::Vector3d>& inputPoints,
                                const std::vector<int>& neighborsIndex,
                                const Eigen::VectorXd& inputDepths,
                                std::vector<Eigen::Vector3d>& output,
                                std::vector<int>& outputIndex);

private:
};
}
