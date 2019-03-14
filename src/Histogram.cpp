/*
 * Histogram.cpp
 *
 *  Created on: Mar 4, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/Histogram.h"
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace Mono_Lidar {
Histogram::Histogram(const double binWitdh, const int binCount) : _binWitdh(binWitdh) {
    // initialize bins
    _bins = std::vector<HistogramBin>(binCount, HistogramBin());
}

void Histogram::AddElement(double value) {
    //    if (_binWitdh == 0.) {
    //        std::cout << "Histogram error: bin width==0." << std::endl;
    //        throw std::runtime_error("Histogram: binWidth==0.");
    //    }
    //    if (_bins.size() == 0) {
    //        std::cout << "Histogram error: bins size=0" << std::endl;
    //        throw std::runtime_error("Histogram error: bins size=0");
    //    }
    // Very big values are put in last bin
    value = std::min(value, 1e10); // if value is very big we will have numerical instabilities
    int binIndex = static_cast<int>(std::min(std::abs(value / _binWitdh), static_cast<double>(_bins.size()) - 1.));

    _bins[binIndex]._binContent.push_back(value);
}

std::vector<double>& Histogram::getElements(int binIndex) {
    //    if (_bins.size() <= binIndex) {
    //        std::cout << "Histogram error: wrong indexing" << std::endl;
    //        throw std::runtime_error("Histogram error: wrong indexing");
    //    }
    return _bins[binIndex]._binContent;
}

int Histogram::binElemCount(int binIndex) {
    //    if (_bins.size() <= binIndex) {
    //        std::cout << "Histogram error: wrong indexing" << std::endl;
    //        throw std::runtime_error("Histogram error: wrong indexing");
    //    }
    return _bins[binIndex]._binContent.size();
}
}
