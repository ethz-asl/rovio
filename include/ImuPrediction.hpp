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

#ifndef IMUPREDICTION_HPP_
#define IMUPREDICTION_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "Prediction.hpp"
#include "State.hpp"
#include "FilterState.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

using namespace LWF;

class PredictionMeas: public State<VectorElement<3>,VectorElement<3>>{
 public:
  static constexpr unsigned int _acc = 0;
  static constexpr unsigned int _gyr = _acc+1;
  PredictionMeas(){
    static_assert(_gyr+1==E_,"Error with indices");
  }
  ~PredictionMeas(){};
};

template<typename STATE>
class PredictionNoise: public State<TH_multiple_elements<VectorElement<3>,5+STATE::nMax_>,
      TH_multiple_elements<ScalarElement,STATE::nMax_>,
      TH_multiple_elements<VectorElement<2>,STATE::nMax_>>{
 public:
  using State<TH_multiple_elements<VectorElement<3>,5+STATE::nMax_>>::E_;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _att = _gyb+1;
  static constexpr unsigned int _dep = _att+1;
  static constexpr unsigned int _nor = _dep+1;
  PredictionNoise(){
    static_assert(_nor+1==E_,"Error with indices");
  }
  ~PredictionNoise(){};
};

template<typename STATE>
class DeevioPrediction: public Prediction<STATE,PredictionMeas,PredictionNoise<STATE>>{
 public:
  typedef Prediction<STATE,PredictionMeas,PredictionNoise<STATE>> Base;
  using Base::eval;
  using Base::prenoiP_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  typedef typename Base::mtCovMat mtCovMat;
  const Eigen::Vector3d g_;
  DeevioPrediction():g_(0,0,-9.81){};
  ~DeevioPrediction(){};
  mtState eval(const mtState& state, const mtMeas& meas, const mtNoise noise, double dt) const{
    mtState output;
    output.template get<mtState::_aux>().MwIMmeas_ = meas.template get<mtMeas::_gyr>();
    output.template get<mtState::_aux>().MwIMest_ = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    Eigen::Vector3d dOmega = -dt*(output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt));
    rot::RotationQuaternionPD dQ;
    dQ = dQ.exponentialMap(dOmega);
    output.template get<mtState::_att>() = state.template get<mtState::_att>()*dQ;
    output.template get<mtState::_pos>() = (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega))
        *state.template get<mtState::_pos>()-dt*(state.template get<mtState::_vel>()-noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.template get<mtState::_vel>() = (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega))
        *state.template get<mtState::_vel>()-dt*(meas.template get<mtMeas::_acc>()-state.template get<mtState::_acb>()+state.template get<mtState::_att>().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.template get<mtState::_acb>() = state.template get<mtState::_acb>()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.template get<mtState::_gyb>() = state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    for(unsigned int i=0;i<mtState::nMax_;i++){
      output.template get<mtState::_dep>(i) = state.template get<mtState::_dep>(i)+dt*(state.template get<mtState::_nor>(i).transpose()*state.template get<mtState::_vel>()+noise.template get<mtNoise::_dep>(i)/sqrt(dt));
//      output.template get<mtState::_nor>(i) = state.template get<mtState::_cla>(i); // todo
    }
    output.template get<mtState::_aux>().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.template get<mtState::_aux>().img_ = state.template get<mtState::_aux>().img_;
    output.fix();
    return output;
  }
  void noMeasCase(mtState& state, mtCovMat& cov, mtMeas& meas, double dt){
    meas.template get<mtMeas::_acc>() = state.template get<mtState::_gyb>();
    meas.template get<mtMeas::_gyr>() = state.template get<mtState::_acb>()-state.template get<mtState::_att>().inverseRotate(g_);
  }
  mtJacInput jacInput(const mtState& state, const mtMeas& meas, double dt) const{
    mtJacInput J;
    Eigen::Vector3d dOmega = -dt*(meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    J.setIdentity(); // Handles clone and calibrations
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) =
        (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega));
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) =
        -dt*Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_gyb>()) =
        -dt*kindr::linear_algebra::getSkewMatrixFromVector(state.template get<mtState::_pos>());
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) =
        (Eigen::Matrix3d::Identity()+kindr::linear_algebra::getSkewMatrixFromVector(dOmega));
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) =
        dt*Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) =
        -dt*kindr::linear_algebra::getSkewMatrixFromVector(state.template get<mtState::_vel>());
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) =
        dt*rot::RotationMatrixPD(state.template get<mtState::_att>()).matrix().transpose()*kindr::linear_algebra::getSkewMatrixFromVector(g_);
    J.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) =
        Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) =
        Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) =
        dt*rot::RotationMatrixPD(state.template get<mtState::_att>()).matrix()*Lmat(dOmega);
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) =
        Eigen::Matrix3d::Identity();
    return J;
  }
  mtJacNoise jacNoise(const mtState& state, const mtMeas& meas, double dt) const{
    mtJacNoise J;
    Eigen::Vector3d dOmega = -dt*(meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    J.setZero();
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_att>()) =
        kindr::linear_algebra::getSkewMatrixFromVector(state.template get<mtState::_pos>())*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        kindr::linear_algebra::getSkewMatrixFromVector(state.template get<mtState::_vel>())*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        -rot::RotationMatrixPD(state.template get<mtState::_att>()).matrix()*Lmat(dOmega)*sqrt(dt);
    if(STATE::calVE_){
      J.template block<3,3>(mtState::template getId<mtState::_vep>(),mtNoise::template getId<mtNoise::_vep>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
      J.template block<3,3>(mtState::template getId<mtState::_vea>(),mtNoise::template getId<mtNoise::_vea>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
    }
    if(STATE::calPI_){
      J.template block<3,3>(mtState::template getId<mtState::_pip>(),mtNoise::template getId<mtNoise::_pip>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
      J.template block<3,3>(mtState::template getId<mtState::_pia>(),mtNoise::template getId<mtNoise::_pia>()) = Eigen::Matrix3d::Identity()*sqrt(dt);
    }
    return J;
  }
};

}


#endif /* IMUPREDICTION_HPP_ */
