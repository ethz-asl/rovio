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

#ifndef ROVIO_FILTERSTATES_HPP_
#define ROVIO_FILTERSTATES_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/FilterState.hpp"
#include <map>
#include <unordered_set>
#include "rovio/common_vision.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

template<unsigned int nMax, int nLevels, int patchSize>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nMax,nLevels,patchSize>>{
 public:
  StateAuxiliary(){
    MwIMest_.setZero();
    MwIMmeas_.setZero();
    wMeasCov_.setIdentity();
    for(unsigned int i=0;i<nMax;i++){
      useInUpdate_[i] = true;
      A_red_[i].setIdentity();
      b_red_[i].setZero();
      bearingMeas_[i].setIdentity();
      bearingCorners_[i][0].setZero();
      bearingCorners_[i][1].setZero();
    }
  };
  ~StateAuxiliary(){};
  V3D MwIMest_;
  V3D MwIMmeas_;
  M3D wMeasCov_;
  bool useInUpdate_[nMax];
  Eigen::Matrix2d A_red_[nMax];
  Eigen::Vector2d b_red_[nMax];
  LWF::NormalVectorElement bearingMeas_[nMax];
  Eigen::Vector2d bearingCorners_[nMax][2];
};

template<unsigned int nMax, int nLevels, int patchSize>
class State: public LWF::State<
    LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
    LWF::TH_multiple_elements<LWF::QuaternionElement,2>,
    LWF::ArrayElement<LWF::ScalarElement,nMax>,
    LWF::ArrayElement<LWF::NormalVectorElement,nMax>,
    StateAuxiliary<nMax,nLevels,patchSize>>{
 public:
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
      LWF::TH_multiple_elements<LWF::QuaternionElement,2>,
      LWF::ArrayElement<LWF::ScalarElement,nMax>,
      LWF::ArrayElement<LWF::NormalVectorElement,nMax>,
      StateAuxiliary<nMax,nLevels,patchSize>> Base;
  using Base::D_;
  using Base::E_;
  static constexpr unsigned int nMax_ = nMax;
  static constexpr unsigned int nLevels_ = nLevels;
  static constexpr unsigned int patchSize_ = patchSize;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _vep = _gyb+1;
  static constexpr unsigned int _att = _vep+1;
  static constexpr unsigned int _vea = _att+1;
  static constexpr unsigned int _dep = _vea+1;
  static constexpr unsigned int _nor = _dep+1;
  static constexpr unsigned int _aux = _nor+1;
  State(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_vep>() = "vep";
    this->template getName<_att>() = "att";
    this->template getName<_vea>() = "vea";
    this->template getName<_dep>() = "dep";
    this->template getName<_nor>() = "nor";
    this->template getName<_aux>() = "auxiliary";
  }
  ~State(){};
};

class PredictionMeas: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _acc = 0;
  static constexpr unsigned int _gyr = _acc+1;
  PredictionMeas(){
    static_assert(_gyr+1==E_,"Error with indices");
    this->template getName<_acc>() = "acc";
    this->template getName<_gyr>() = "gyr";
  }
  ~PredictionMeas(){};
};

template<typename STATE>
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,7>,
      LWF::ArrayElement<LWF::ScalarElement,STATE::nMax_>,
      LWF::ArrayElement<LWF::VectorElement<2>,STATE::nMax_>>{
 public:
  using LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,7>,
      LWF::ArrayElement<LWF::ScalarElement,STATE::nMax_>,
      LWF::ArrayElement<LWF::VectorElement<2>,STATE::nMax_>>::E_;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _vep = _gyb+1;
  static constexpr unsigned int _att = _vep+1;
  static constexpr unsigned int _vea = _att+1;
  static constexpr unsigned int _dep = _vea+1;
  static constexpr unsigned int _nor = _dep+1;
  PredictionNoise(){
    static_assert(_nor+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_vep>() = "vep";
    this->template getName<_att>() = "att";
    this->template getName<_vea>() = "vea";
    this->template getName<_dep>() = "dep";
    this->template getName<_nor>() = "nor";
  }
  ~PredictionNoise(){};
};

template<unsigned int nMax, int nLevels, int patchSize>
class FilterState: public LWF::FilterState<State<nMax,nLevels,patchSize>,PredictionMeas,PredictionNoise<State<nMax,nLevels,patchSize>>,0,false>{
 public:
  typedef LWF::FilterState<State<nMax,nLevels,patchSize>,PredictionMeas,PredictionNoise<State<nMax,nLevels,patchSize>>,0,false> Base;
  typedef typename Base::mtState mtState;
  using Base::state_;
  using Base::cov_;
  using Base::usePredictionMerge_;
  FeatureManager<nLevels,patchSize,nMax> fManager_;
  cv::Mat img_; // Mainly used for drawing
  cv::Mat patchDrawing_; // Mainly used for drawing
  double imgTime_;
  int imageCounter_;
  FilterState(){
    usePredictionMerge_ = true;
    imgTime_ = 0.0;
    imageCounter_ = 0;
  }
  void initWithImuPose(V3D IrIM, QPD qMI){
    state_.template get<mtState::_pos>() = qMI.rotate(IrIM);
    state_.template get<mtState::_att>() = qMI.inverted();
  }
  void initWithAccelerometer(const V3D& fMeasInit){
    V3D unitZ(0,0,1);
    if(fMeasInit.norm()>1e-6){
      state_.template get<mtState::_att>().setFromVectors(unitZ,fMeasInit);
    } else {
      state_.template get<mtState::_att>().setIdentity();
    }
  }
  void initializeFeatureState(unsigned int i, V3D n, double d,const Eigen::Matrix<double,3,3>& initCov){
    state_.template get<mtState::_dep>(i) = d;
    state_.template get<mtState::_nor>(i).setFromVector(n);
    cov_.template block<mtState::D_,1>(0,mtState::template getId<mtState::_dep>(i)).setZero();
    cov_.template block<1,mtState::D_>(mtState::template getId<mtState::_dep>(i),0).setZero();
    cov_.template block<mtState::D_,2>(0,mtState::template getId<mtState::_nor>(i)).setZero();
    cov_.template block<2,mtState::D_>(mtState::template getId<mtState::_nor>(i),0).setZero();
    cov_.template block<1,1>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)) = initCov.block<1,1>(0,0);
    cov_.template block<1,2>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_nor>(i)) = initCov.block<1,2>(0,1);
    cov_.template block<2,1>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_dep>(i)) = initCov.block<2,1>(1,0);
    cov_.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)) = initCov.block<2,2>(1,1);
  }
  void removeFeature(unsigned int i){
    state_.template get<mtState::_dep>(i) = 1.0;
    state_.template get<mtState::_nor>(i).setIdentity();
    cov_.template block<mtState::D_,1>(0,mtState::template getId<mtState::_dep>(i)).setZero();
    cov_.template block<1,mtState::D_>(mtState::template getId<mtState::_dep>(i),0).setZero();
    cov_.template block<mtState::D_,2>(0,mtState::template getId<mtState::_nor>(i)).setZero();
    cov_.template block<2,mtState::D_>(mtState::template getId<mtState::_nor>(i),0).setZero();
    cov_.template block<1,1>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)).setIdentity();
    cov_.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)).setIdentity();
  }
};

}


#endif /* ROVIO_FILTERSTATES_HPP_ */
