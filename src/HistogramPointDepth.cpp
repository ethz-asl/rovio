/*
 * PointHistogram.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: wilczynski
 */

#include <math.h>

#include "monolidar_fusion/DepthCalcHistPoint.h"
#include "monolidar_fusion/HistogramPointDepth.h"

namespace Mono_Lidar {

bool PointHistogram::FilterPointsMinDistBlob(const std::vector<Eigen::Vector3d>& inputPoints,
                                             const std::vector<int>& neighborsIndex,
                                             const Eigen::VectorXd& inputDepths,
                                             const double binWitdh,
                                             const int minimalMaximumSize,
                                             std::vector<Eigen::Vector3d>& output,
                                             std::vector<int>& outputIndex,
                                             double& lowerBorder,
                                             double& higherBorder,
                                             const std::shared_ptr<DepthCalcStatsSinglePoint>& debugStats) {
    // check precondition
    if (inputPoints.size() != inputDepths.rows()) {
        throw "ERROR IN PointHistogram::FilterPointsMinDistBlob: size of inputPoints and inputDepths are uneqal.";
    }
    // Get Borders of the first local maxium bin
    lowerBorder = -1;
    higherBorder = -1;
    int binCount;

    // get the max distance and create histogram
    int depthCount = inputDepths.rows();
    int maxDist = 0;

    for (int i = 0; i < depthCount; i++) {
        if (inputDepths(i) > maxDist)
            maxDist = ceil(inputDepths(i));
    }

    binCount = (maxDist) / binWitdh + 1;

    // debug log
    if (debugStats != NULL) {
        debugStats->_histMinDist = 0;
        debugStats->_histMaxDist = maxDist;
        debugStats->_histBinWitdh = binWitdh;
        debugStats->_histBinCount = binCount;
    }

    if (binCount <= 1) {
        return false;
    }
    Histogram histogram(binWitdh, binCount);

    // Fill the histogram with values

    for (int i = 0; i < depthCount; i++) {
        double depth = inputDepths[i];
        histogram.AddElement(depth);
    }

    // get the local maximum
    int binMaxId = -1;
    int binMaxVal = -1;
    int binValue = 0;

    for (int i = 0; i < binCount; i++) {
        float lastBinValue = binValue;
        binValue = histogram.binElemCount(i);


        if ((binValue > binMaxVal) && (binValue >= minimalMaximumSize)) {
            binMaxVal = binValue;
            binMaxId = i;
        } else if (binValue < binMaxVal)
            // first local maximum found
            break;

        // cancel if single points are detected which are under the minimum count
        if ((lastBinValue > 0) && (binValue == 0))
            return false;
    }

    // debug log
    if (debugStats != NULL) {
        for (int i = 0; i < binCount; i++) {
            float binValue = histogram.binElemCount(i);
            debugStats->_histDepthEntryCount.push_back(binValue);
        }
    }

    if (binMaxId < 0) {
        return false;
    }

    lowerBorder = binMaxId * binWitdh - 0.0f * binWitdh;
    higherBorder = (binMaxId)*binWitdh + 1.0f * binWitdh;

    // debug log
    if (debugStats != NULL) {
        debugStats->_histLowerBorder = lowerBorder;
        debugStats->_histHigherBorder = higherBorder;
    }

    if (debugStats != NULL)
        debugStats->_histFound = 1;

    // OUPUT
    // Filter all points which are in the determined bin
    output.clear();
    int pointSize = inputPoints.size();
    for (int i = 0; i < pointSize; i++) {
        if ((inputDepths[i] >= lowerBorder) && (inputDepths[i] < higherBorder)) {
            output.push_back(inputPoints.at(i));
            outputIndex.push_back(neighborsIndex.at(i));
        }
    }

    return true;
}

bool PointHistogram::GetNearestPoint(const std::vector<Eigen::Vector3d>& inputPoints,
                                     const std::vector<int>& neighborsIndex,
                                     const Eigen::VectorXd& inputDepths,
                                     std::vector<Eigen::Vector3d>& output,
                                     std::vector<int>& outputIndex) {
    float minDepth = std::numeric_limits<double>::max();
    int minIndex = -1;
    int depthCount = inputDepths.rows();

    for (int i = 0; i < depthCount; i++) {
        if (inputDepths(i) < minDepth) {
            minDepth = inputDepths(i);
            minIndex = neighborsIndex.at(i);
        }
    }

    // check if no point available
    if (minIndex == -1)
        return false;

    // nearest point has been found
    output.push_back(inputPoints.at(minIndex));
    outputIndex.push_back(minIndex);

    return true;
}
}
