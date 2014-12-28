/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_FILTER_HPP_
#define ROVIO_FILTER_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "FilterBase.hpp"
#include "State.hpp"
#include "FilterState.hpp"
#include "ImgUpdate.hpp"
#include "ImuPrediction.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

using namespace LWF;

template<typename STATE>
class Filter:public FilterBase<ImuPrediction<STATE>,ImgUpdate<STATE>>{
 public:
  typedef FilterBase<ImuPrediction<STATE>,ImgUpdate<STATE>> Base;
  using Base::init_;
  using Base::reset;
  using Base::predictionTimeline_;
  using Base::safe_;
  using Base::front_;
  using Base::readFromInfo;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtPrediction mtPrediction;
  Filter(){
    readFromInfo("rovio.info");
    std::get<0>(init_.outlierDetectionTuple_).setEnabledAll(true);
    reset(0.0);
  }
  ~Filter(){};
//  void resetToImuPose(Eigen::Vector3d IrIM, rot::RotationQuaternionPD qMI, double t = 0.0){
//    init_.state_.initWithImuPose(IrIM,qMI);
//    reset(t);
//  }
//  void resetWithAccelerometer(const Eigen::Vector3d& fMeasInit, double t = 0.0){
//    init_.state_.initWithAccelerometer(fMeasInit);
//    reset(t);
//  }
//  void resetToKeyframe(double t = 0.0) {
//    std::cout << "Reseting to keyframe" << std::endl;
//    double imuMeasTime = 0.0;
//    if(predictionTimeline_.getNextTime(t,imuMeasTime)){  // Find close accelerometer measurement
//      resetWithAccelerometer(predictionTimeline_.measMap_[imuMeasTime].template get<mtPrediction::mtMeas::_acc>(),t); // Initialize with accelerometer
//    } else {
//      reset(t);
//    }
//    safe_.state_.cloneCurrentToKeyframe(safe_.cov_,t);
//    front_ = safe_;
//  }
};

}


#endif /* ROVIO_FILTER_HPP_ */
