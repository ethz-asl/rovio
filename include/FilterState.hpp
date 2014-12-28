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
#include "State.hpp"
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <unordered_set>

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

using namespace LWF;

template<unsigned int nMax>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nMax>>{
 public:
  StateAuxiliary(){
    MwIMest_.setZero();
    MwIMmeas_.setZero();
    wMeasCov_.setIdentity();
    for(unsigned int i=0;i<nMax;i++){
      ID_[i] = 0;
      countSinceVisible_[i] = 0;
      isVisible_[i] = false;
      indEmpty_.insert(i);
    }
  };
  ~StateAuxiliary(){};
  cv::Mat img_;
  std::map<unsigned int,unsigned int> indFeature_;
  std::unordered_set<unsigned int> indEmpty_;
  unsigned int ID_[nMax];
  bool isVisible_[nMax];
  unsigned int countSinceVisible_[nMax];
  Eigen::Vector3d MwIMest_;
  Eigen::Vector3d MwIMmeas_;
  Eigen::Matrix3d wMeasCov_;
  void addIndex(unsigned int ID){
    if(indEmpty_.empty()){
      std::cout << "STATE: maximal number of feature reached" << std::endl;
    } else {
      ID_[*indEmpty_.begin()] = ID;
      indFeature_[ID] = *indEmpty_.begin();
      countSinceVisible_[indFeature_[ID]] = 0;
      isVisible_[indFeature_[ID]] = false;
      indEmpty_.erase(indEmpty_.begin());
    }
  }
  void removeIndex(unsigned int ID){
    ID_[indFeature_.at(ID)] = 0;
    indEmpty_.insert(indFeature_.at(ID));
    indFeature_.erase (ID);
  }
  void resetVisible(){
    for(unsigned int i=0;i<nMax;i++){
      isVisible_[i] = false;
    }
  }
  void countSinceVisible(){
    for(unsigned int i=0;i<nMax;i++){
      if(isVisible_[i] == false){
        countSinceVisible_[i]++;
      } else {
        countSinceVisible_[i] = 0;
      }
    }
  }
};

template<unsigned int nMax>
class FilterState: public State<
    TH_multiple_elements<VectorElement<3>,4>,
    QuaternionElement,
    ArrayElement<ScalarElement,nMax>,
    ArrayElement<NormalVectorElement,nMax>,
    StateAuxiliary<nMax>>{
 public:
  typedef State<
      TH_multiple_elements<VectorElement<3>,4>,
      QuaternionElement,
      ArrayElement<ScalarElement,nMax>,
      ArrayElement<NormalVectorElement,nMax>,
      StateAuxiliary<nMax>> Base;
  typedef typename Base::mtCovMat mtCovMat;
  using Base::D_;
  using Base::E_;
  static constexpr unsigned int nMax_ = nMax;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _att = _gyb+1;
  static constexpr unsigned int _dep = _att+1;
  static constexpr unsigned int _nor = _dep+1;
  static constexpr unsigned int _aux = _nor+1;
  FilterState(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_att>() = "att";
    this->template getName<_dep>() = "dep";
    this->template getName<_nor>() = "nor";
    this->template getName<_aux>() = "auxiliary";
  }
  ~FilterState(){};
  void initWithImuPose(Eigen::Vector3d IrIM, rot::RotationQuaternionPD qMI){
    this->template get<_pos>() = qMI.rotate(IrIM);
    this->template get<_att>() = qMI.inverted();
  }
  void initWithAccelerometer(const Eigen::Vector3d& fMeasInit){
    Eigen::Vector3d unitZ(0,0,1);
    if(fMeasInit.norm()>1e-6){
      this->template get<_att>().setFromVectors(fMeasInit,unitZ); // TODO: check
    } else {
      this->template get<_att>().setIdentity();
    }
  }
  void initializeFeature(mtCovMat& stateCov, unsigned int i, Eigen::Vector3d n, double d,const Eigen::Matrix<double,3,3>& initCov){
    this->template get<_dep>(i) = d;
    this->template get<_nor>(i) = n;
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
