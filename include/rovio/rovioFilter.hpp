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

#ifndef ROVIO_ROVIO_FILTER_HPP_
#define ROVIO_ROVIO_FILTER_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/FilterBase.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/ImgUpdate.hpp"
#include "rovio/ImuPrediction.hpp"
#include "rovio/Camera.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

template<typename FILTERSTATE>
class RovioFilter:public LWF::FilterBase<ImuPrediction<FILTERSTATE>,ImgUpdate<FILTERSTATE>>{
 public:
  typedef LWF::FilterBase<ImuPrediction<FILTERSTATE>,ImgUpdate<FILTERSTATE>> Base;
  using Base::init_;
  using Base::reset;
  using Base::predictionTimeline_;
  using Base::safe_;
  using Base::front_;
  using Base::readFromInfo;
  using Base::boolRegister_;
  using Base::intRegister_;
  using Base::doubleRegister_;
  using Base::mUpdates_;
  using Base::mPrediction_;
  using Base::stringRegister_;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtPrediction mtPrediction;
  typedef typename Base::mtState mtState;
  rovio::Camera camera_;
  std::string cameraCalibrationFile_;
  RovioFilter(){
    std::get<0>(mUpdates_).setCamera(&camera_);
    cameraCalibrationFile_ = "calib.yaml";
    stringRegister_.registerScalar("cameraCalibrationFile",cameraCalibrationFile_);
    boolRegister_.registerScalar("doVECalibration",init_.state_.template get<mtState::_aux>().doVECalibration_);
    intRegister_.registerScalar("depthType",init_.state_.template get<mtState::_aux>().depthTypeInt_);
    doubleRegister_.registerVector("MrMV",init_.state_.template get<mtState::_aux>().MrMV_);
    doubleRegister_.registerQuaternion("qVM",init_.state_.template get<mtState::_aux>().qVM_);
    int ind;
    for(int i=0;i<FILTERSTATE::mtState::nMax_;i++){
      ind = mtState::template getId<mtState::_nor>(i);
      doubleRegister_.removeScalarByVar(init_.cov_(ind,ind));
      doubleRegister_.removeScalarByVar(init_.cov_(ind+1,ind+1));
      ind = mtState::template getId<mtState::_dep>(i);
      doubleRegister_.removeScalarByVar(init_.cov_(ind,ind));
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_dep>(i));
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_nor>(i).q_.toImplementation().w());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_nor>(i).q_.toImplementation().x());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_nor>(i).q_.toImplementation().y());
      doubleRegister_.removeScalarByVar(init_.state_.template get<mtState::_nor>(i).q_.toImplementation().z());
      std::get<0>(mUpdates_).doubleRegister_.removeScalarByVar(std::get<0>(mUpdates_).outlierDetection_.getMahalTh(i));
      std::get<0>(mUpdates_).doubleRegister_.registerScalar("MahalanobisTh",std::get<0>(mUpdates_).outlierDetection_.getMahalTh(i));
    }
    std::get<0>(mUpdates_).outlierDetection_.setEnabledAll(true);
    reset(0.0);
  }
  void refreshProperties(){
    camera_.load(cameraCalibrationFile_);
    init_.state_.template get<mtState::_aux>().depthMap_.setType(init_.state_.template get<mtState::_aux>().depthTypeInt_);
  };
  ~RovioFilter(){};
//  void resetToImuPose(V3D IrIM, QPD qMI, double t = 0.0){
//    init_.state_.initWithImuPose(IrIM,qMI);
//    reset(t);
//  }
  void resetWithAccelerometer(const V3D& fMeasInit, double t = 0.0){
    init_.initWithAccelerometer(fMeasInit);
    reset(t);
  }
  void setExtrinsics(const Eigen::Matrix3d& R_VM, const Eigen::Vector3d& VrVM){
    rot::RotationMatrixAD R(R_VM);
    init_.state_.template get<mtState::_aux>().qVM_ = QPD(R.getPassive());
    init_.state_.template get<mtState::_aux>().MrMV_ = -init_.state_.template get<mtState::_aux>().qVM_.inverseRotate(VrVM); // TODO: all filterstates
  }
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


#endif /* ROVIO_ROVIO_FILTER_HPP_ */
