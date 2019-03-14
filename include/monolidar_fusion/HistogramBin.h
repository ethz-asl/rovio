/*
 * HistogramBin.h
 *
 *  Created on: Mar 4, 2017
 *      Author: wilczynski
 */

#pragma once

#include <vector>

namespace Mono_Lidar {
struct HistogramBin {
    std::vector<double> _binContent{};
};
}
