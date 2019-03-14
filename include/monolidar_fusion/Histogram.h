/*
 * Histogram.h
 *
 *  Created on: Mar 4, 2017
 *      Author: wilczynski
 */

#pragma once

#include <vector>

#include "HistogramBin.h"

namespace Mono_Lidar {
class Histogram {
public:
    Histogram(const double binWitdh, const int binCount);

    void AddElement(const double value);
    std::vector<double>& getElements(int binIndex);
    int binElemCount(int binIndex);

private:
    double _binWitdh;
    std::vector<HistogramBin> _bins{};
};
}
