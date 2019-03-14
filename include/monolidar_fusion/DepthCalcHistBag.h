/*
 * DepthCalcHistBag.h
 *
 *  Created on: Jan 18, 2017
 *      Author: wilczynski
 */

#pragma once

#include <memory>
#include <vector>

#include "DepthCalcHistPoint.h"

namespace Mono_Lidar {
struct DepthCalcHistBag {

    double _binBorderLow;
    double _binBorderHigh;
    double _binWitdh;
    int binCount;
    std::vector<std::shared_ptr<DepthCalcHistPoint>> _points;
};
}
