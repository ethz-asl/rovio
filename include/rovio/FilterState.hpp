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

#ifndef FILTERSTATE_HPP_
#define FILTERSTATE_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/FilterState.hpp"
#include <map>
#include <unordered_set>

#include "common_vision.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

using namespace LWF;

// TODO remove when including in LWFM
typedef rot::RotationQuaternionPD QPD;
typedef rot::RotationMatrixPD MPD;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
static M3D gSM(const V3D& vec){
  return kindr::linear_algebra::getSkewMatrixFromVector(vec);
}

template<unsigned int nMax, int nLevels, int patchSize>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nMax,nLevels,patchSize>>{
 public:
  StateAuxiliary(){
    imgTime_ = 0.0;
    MwIMest_.setZero();
    MwIMmeas_.setZero();
    wMeasCov_.setIdentity();
    imageCounter_ = 0;
  };
  ~StateAuxiliary(){};
  FeatureManager<nLevels,patchSize,nMax> fManager_;
  cv::Mat img_; // Mainly used for drawing
  double imgTime_;
  LWF::NormalVectorElement corners_[nMax][2];
  V3D MwIMest_;
  V3D MwIMmeas_;
  M3D wMeasCov_;
  int imageCounter_;
};

template<unsigned int nMax, int nLevels, int patchSize>
class FilterState: public State<
    TH_multiple_elements<VectorElement<3>,5>,
    TH_multiple_elements<QuaternionElement,2>,
    ArrayElement<ScalarElement,nMax>,
    ArrayElement<NormalVectorElement,nMax>,
    StateAuxiliary<nMax,nLevels,patchSize>>{
 public:
  typedef State<
      TH_multiple_elements<VectorElement<3>,5>,
      TH_multiple_elements<QuaternionElement,2>,
      ArrayElement<ScalarElement,nMax>,
      ArrayElement<NormalVectorElement,nMax>,
      StateAuxiliary<nMax,nLevels,patchSize>> Base;
  typedef typename Base::mtCovMat mtCovMat;
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
  FilterState(){
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
  ~FilterState(){};
  void initWithImuPose(V3D IrIM, QPD qMI){
    this->template get<_pos>() = qMI.rotate(IrIM);
    this->template get<_att>() = qMI.inverted();
  }
  void initWithAccelerometer(const V3D& fMeasInit){
    V3D unitZ(0,0,1);
    if(fMeasInit.norm()>1e-6){
      this->template get<_att>().setFromVectors(unitZ,fMeasInit);
    } else {
      this->template get<_att>().setIdentity();
    }
  }
  void initializeFeatureState(mtCovMat& stateCov, unsigned int i, V3D n, double d,const Eigen::Matrix<double,3,3>& initCov){
    this->template get<_dep>(i) = d;
    this->template get<_nor>(i).setFromVector(n);
    stateCov.template block<D_,1>(0,this->template getId<_dep>(i)).setZero();
    stateCov.template block<1,D_>(this->template getId<_dep>(i),0).setZero();
    stateCov.template block<D_,2>(0,this->template getId<_nor>(i)).setZero();
    stateCov.template block<2,D_>(this->template getId<_nor>(i),0).setZero();
    stateCov.template block<1,1>(this->template getId<_dep>(i),this->template getId<_dep>(i)) = initCov.block<1,1>(0,0);
    stateCov.template block<1,2>(this->template getId<_dep>(i),this->template getId<_nor>(i)) = initCov.block<1,2>(0,1);
    stateCov.template block<2,1>(this->template getId<_nor>(i),this->template getId<_dep>(i)) = initCov.block<2,1>(1,0);
    stateCov.template block<2,2>(this->template getId<_nor>(i),this->template getId<_nor>(i)) = initCov.block<2,2>(1,1);
  }
};

}


#endif /* FILTERSTATE_HPP_ */
