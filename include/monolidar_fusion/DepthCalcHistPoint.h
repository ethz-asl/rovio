/*
 * DepthCalcHistPoint.h
 *
 *  Created on: Jan 18, 2017
 *      Author: wilczynski
 */

#pragma once

#include <memory>

namespace Mono_Lidar {
// Coordinates of the given point in camera cos
struct DepthCalcHistPoint {
    float _camX;
    float _camY;
    float _camZ;
    float _camDist;
};
}
