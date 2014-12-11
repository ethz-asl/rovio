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

#ifndef POSEUPDATE_HPP_
#define POSEUPDATE_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "Update.hpp"
#include "State.hpp"
#include "FilterState.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace deevio {

using namespace LWF;

class PoseInnovation: public State<VectorElement<3>,QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseInnovation(){
    static_assert(_att+1==E_,"Error with indices");
  };
  ~PoseInnovation(){};
};
class PoseUpdateMeas: public State<VectorElement<3>,QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateMeas(){
    static_assert(_att+1==E_,"Error with indices");
  };
  ~PoseUpdateMeas(){};
};
class PoseUpdateNoise: public State<VectorElement<3>,VectorElement<3>>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateNoise(){
    static_assert(_att+1==E_,"Error with indices");
  };
  ~PoseUpdateNoise(){};
};
class PoseOutlierDetection: public OutlierDetection<
    ODEntry<PoseInnovation::template getId<PoseInnovation::_pos>(),6>>{
};

template<typename STATE>
class PoseUpdate: public Update<PoseInnovation,STATE,PoseUpdateMeas,PoseUpdateNoise,
    PoseOutlierDetection,DummyPrediction,false>{
 public:
  typedef Update<PoseInnovation,STATE,PoseUpdateMeas,PoseUpdateNoise,
      PoseOutlierDetection,DummyPrediction,false> Base;
  using typename Base::eval;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtCovMat mtCovMat;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  PoseUpdate(){
    static_assert(STATE::calPI_,"PoseUpdate requires enabling of calibration");
    qVM_.setIdentity();
    MrMV_.setZero();
  };
  ~PoseUpdate(){};
  rot::RotationQuaternionPD qVM_;
  Eigen::Vector3d MrMV_;
  mtInnovation eval(const mtState& state, const mtMeas& meas, const mtNoise noise, double dt = 0.0) const{
    mtInnovation y;
    /* Relative 6DOF measurements
     * JrJV = JrJI + qIJ^T*qIM*(MrIM+MrMV_)
     * qVJ = qVM_*qIM^T*qIJ
     */
    y.template get<mtInnovation::_pos>() = state.template get<mtState::_pip>() + (state.template get<mtState::_pia>().inverted()*state.template get<mtState::_att>()).rotate(
        Eigen::Vector3d(state.template get<mtState::_pos>()+MrMV_)) - meas.template get<mtMeas::_pos>() + noise.template get<mtNoise::_pos>();
    rot::RotationQuaternionPD attNoise = attNoise.exponentialMap(noise.template get<mtNoise::_att>());
    y.template get<mtInnovation::_att>() = attNoise*qVM_*state.template get<mtState::_att>().inverted()*state.template get<mtState::_pia>()
        *meas.template get<mtMeas::_att>().inverted();
    return y;
  }
  mtJacInput jacInput(const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    mtJacInput J;
    J.setZero();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) =
          rot::RotationMatrixPD(state.template get<mtState::_pia>().inverted()*state.template get<mtState::_att>()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pip>()) =
                Eigen::Matrix3d::Identity();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_att>()) =
          rot::RotationMatrixPD(state.template get<mtState::_pia>().inverted()).matrix()*
          kindr::linear_algebra::getSkewMatrixFromVector(state.template get<mtState::_att>().rotate(Eigen::Vector3d(state.template get<mtState::_pos>()+MrMV_)));
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pia>()) =
          -kindr::linear_algebra::getSkewMatrixFromVector((state.template get<mtState::_pia>().inverted()*state.template get<mtState::_att>()).rotate(
              Eigen::Vector3d(state.template get<mtState::_pos>()+MrMV_)))*rot::RotationMatrixPD(state.template get<mtState::_pia>().inverted()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_att>()) =
          -rot::RotationMatrixPD(qVM_*state.template get<mtState::_att>().inverted()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_pia>()) =
          rot::RotationMatrixPD(qVM_*state.template get<mtState::_att>().inverted()).matrix();
    return J;
  }
  mtJacNoise jacNoise(const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    mtJacNoise J;
    J.setZero();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtNoise::template getId<mtNoise::_att>()) = Eigen::Matrix3d::Identity();
    return J;
  }
};

}


#endif /* POSEUPDATE_HPP_ */
