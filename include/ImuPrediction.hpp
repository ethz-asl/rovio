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
class PredictionNoise: public State<TH_multiple_elements<VectorElement<3>,7>,
      ArrayElement<ScalarElement,STATE::nMax_>,
      ArrayElement<VectorElement<2>,STATE::nMax_>>{
 public:
  using State<TH_multiple_elements<VectorElement<3>,7>,
      ArrayElement<ScalarElement,STATE::nMax_>,
      ArrayElement<VectorElement<2>,STATE::nMax_>>::E_;
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

template<typename STATE>
class ImuPrediction: public Prediction<STATE,PredictionMeas,PredictionNoise<STATE>>{
 public:
  typedef Prediction<STATE,PredictionMeas,PredictionNoise<STATE>> Base;
  using Base::eval;
  using Base::prenoiP_;
  using Base::doubleRegister_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  typedef typename Base::mtCovMat mtCovMat;
  const V3D g_;
  QPD qVM_;
  V3D MrMV_;
  bool doVECalibration;
  double sign; // TODO: delete
  ImuPrediction():g_(0,0,-9.81), Base(false){
    qVM_.setIdentity();
    MrMV_.setZero();
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
    int ind;
    doVECalibration = true;
    sign = -1.0; // TODO: why the hell is this -1?????????????????
    for(int i=0;i<STATE::nMax_;i++){
      ind = mtNoise::template getId<mtNoise::_nor>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind,ind));
      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind+1,ind+1));
      ind = mtNoise::template getId<mtNoise::_dep>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
      doubleRegister_.registerScalar("PredictionNoise.dep",prenoiP_(ind,ind));
    }
  };
  ~ImuPrediction(){};
  mtState eval(const mtState& state, const mtMeas& meas, const mtNoise noise, double dt) const{
    mtState output;
    output.template get<mtState::_aux>().MwIMmeas_ = meas.template get<mtMeas::_gyr>();
    output.template get<mtState::_aux>().MwIMest_ = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    V3D dOmega = -dt*(output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt)); // TODO: clean
    QPD dQ;
    dQ = dQ.exponentialMap(dOmega);
    V3D imuRor = output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt);
    QPD qVM = qVM_;
    V3D MrMV = MrMV_;
    if(doVECalibration){
      qVM = state.template get<mtState::_vea>();
      MrMV = state.template get<mtState::_vep>();
    }
    V3D camRor = qVM.rotate(imuRor);
    V3D camVel = qVM.rotate(V3D(imuRor.cross(MrMV)-state.template get<mtState::_vel>()));

    output.template get<mtState::_att>() = state.template get<mtState::_att>()*dQ;
    output.template get<mtState::_pos>() = (M3D::Identity()+gSM(dOmega))*state.template get<mtState::_pos>()
        -dt*(state.template get<mtState::_vel>()-noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.template get<mtState::_vel>() = (M3D::Identity()+gSM(dOmega))*state.template get<mtState::_vel>()
        -dt*(meas.template get<mtMeas::_acc>()-state.template get<mtState::_acb>()+state.template get<mtState::_att>().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.template get<mtState::_acb>() = state.template get<mtState::_acb>()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.template get<mtState::_gyb>() = state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    output.template get<mtState::_vep>() = state.template get<mtState::_vep>()+noise.template get<mtNoise::_vep>()*sqrt(dt);
    dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>()*sqrt(dt));
    output.template get<mtState::_vea>() = dQ*state.template get<mtState::_vea>();
    const FeatureManager<4,8,mtState::nMax_>& fManager = state.template get<mtState::_aux>().fManager_;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      output.template get<mtState::_dep>(ind) = state.template get<mtState::_dep>(ind)+dt*pow(state.template get<mtState::_dep>(ind),2)
          *state.template get<mtState::_nor>(ind).getVec().transpose()*camVel + noise.template get<mtNoise::_dep>(ind)*sqrt(dt);
      V3D dm = dt*(gSM(state.template get<mtState::_nor>(ind).getVec())*camVel*state.template get<mtState::_dep>(ind)
          - sign*(M3D::Identity()-state.template get<mtState::_nor>(ind).getVec()*state.template get<mtState::_nor>(ind).getVec().transpose())*camRor)
          + state.template get<mtState::_nor>(ind).getN()*noise.template get<mtNoise::_nor>(ind)*sqrt(dt);
      QPD qm = qm.exponentialMap(dm);
      output.template get<mtState::_nor>(ind) = state.template get<mtState::_nor>(ind).rotated(qm);
//      output.template get<mtState::_dep>(ind) = state.template get<mtState::_dep>(ind); // TODO: delete (+ 2 in bottom)
//      output.template get<mtState::_nor>(ind) = state.template get<mtState::_nor>(ind);

//      std::cout << "Normal Vector: " << state.template get<mtState::_nor>(ind).getVec().transpose() << std::endl;
//      std::cout << "Depth: " << state.template get<mtState::_dep>(ind) << std::endl;
//      std::cout << "Method 1:" << (output.template get<mtState::_nor>(ind).getVec()-state.template get<mtState::_nor>(ind).getVec()).transpose() << std::endl;
//      std::cout << (output.template get<mtState::_dep>(ind)-state.template get<mtState::_dep>(ind)) << std::endl;
//      // P = qIM*(MrIM + MrMV + qVM^T*m*l)
//      V3D point = state.template get<mtState::_att>().rotate(V3D(state.template get<mtState::_pos>()+MrMV
//          +qVM.inverseRotate(V3D(state.template get<mtState::_nor>(ind).getVec()/state.template get<mtState::_dep>(ind)))));
//      // qVM*(qIM^T*P - MrIM - MrMV) = m*l
//      V3D bearing = qVM.rotate(V3D(output.template get<mtState::_att>().inverseRotate(point)-output.template get<mtState::_pos>()-MrMV));
//      double invDepth = 1/bearing.norm();
//      bearing = bearing*invDepth;
//      std::cout << "Method 2:" << (bearing-state.template get<mtState::_nor>(ind).getVec()).transpose() << std::endl;
//      std::cout << (invDepth-state.template get<mtState::_dep>(ind)) << std::endl;
    }
    output.template get<mtState::_aux>().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.template get<mtState::_aux>().img_ = state.template get<mtState::_aux>().img_;
    output.template get<mtState::_aux>().imgTime_ = state.template get<mtState::_aux>().imgTime_;
    output.template get<mtState::_aux>().fManager_ = state.template get<mtState::_aux>().fManager_;




    // todo: change to avoid copy
    output.fix();
    return output;
  }
  void noMeasCase(mtState& state, mtCovMat& cov, mtMeas& meas, double dt){
    meas.template get<mtMeas::_gyr>() = state.template get<mtState::_gyb>();
    meas.template get<mtMeas::_acc>() = state.template get<mtState::_acb>()-state.template get<mtState::_att>().inverseRotate(g_);
  }
  mtJacInput jacInput(const mtState& state, const mtMeas& meas, double dt) const{
    mtJacInput J;
    V3D dOmega = -dt*(meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    V3D imuRor = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    QPD qVM = qVM_;
    V3D MrMV = MrMV_;
    if(doVECalibration){
      qVM = state.template get<mtState::_vea>();
      MrMV = state.template get<mtState::_vep>();
    }
    V3D camRor = qVM.rotate(imuRor);
    V3D camVel = qVM.rotate(V3D(imuRor.cross(MrMV)-state.template get<mtState::_vel>()));
    J.setZero();
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) = (M3D::Identity()+gSM(dOmega));
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) = -dt*M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.template get<mtState::_pos>());
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) = (M3D::Identity()+gSM(dOmega));
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) = dt*M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.template get<mtState::_vel>());
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) = dt*MPD(state.template get<mtState::_att>()).matrix().transpose()*gSM(g_);
    J.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) = M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) = dt*MPD(state.template get<mtState::_att>()).matrix()*Lmat(dOmega);
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) = M3D::Identity();
    NormalVectorElement nOut;
    const FeatureManager<4,8,mtState::nMax_>& fManager = state.template get<mtState::_aux>().fManager_;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      V3D dm = dt*(gSM(state.template get<mtState::_nor>(ind).getVec())*camVel*state.template get<mtState::_dep>(ind)
          - sign*(M3D::Identity()-state.template get<mtState::_nor>(ind).getVec()*state.template get<mtState::_nor>(ind).getVec().transpose())*camRor);
      QPD qm = qm.exponentialMap(dm);
      nOut = state.template get<mtState::_nor>(ind).rotated(qm);
      J(mtState::template getId<mtState::_dep>(ind),mtState::template getId<mtState::_dep>(ind)) = 1.0 + (2.0*dt*state.template get<mtState::_dep>(ind)
              *state.template get<mtState::_nor>(ind).getVec().transpose()*camVel);
      J.template block<1,3>(mtState::template getId<mtState::_dep>(ind),mtState::template getId<mtState::_vel>()) =
          -dt*pow(state.template get<mtState::_dep>(ind),2)*state.template get<mtState::_nor>(ind).getVec().transpose()*MPD(qVM).matrix();
      J.template block<1,3>(mtState::template getId<mtState::_dep>(ind),mtState::template getId<mtState::_gyb>()) =
          dt*pow(state.template get<mtState::_dep>(ind),2)*state.template get<mtState::_nor>(ind).getVec().transpose()*gSM(qVM.rotate(MrMV))*MPD(qVM).matrix();
      J.template block<1,2>(mtState::template getId<mtState::_dep>(ind),mtState::template getId<mtState::_nor>(ind)) =
          dt*pow(state.template get<mtState::_dep>(ind),2)*camVel.transpose()*state.template get<mtState::_nor>(ind).getM();
      J.template block<2,2>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_nor>(ind)) =
          nOut.getM().transpose()*(
                  dt*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)*(
                      -state.template get<mtState::_dep>(ind)*gSM(camVel)
                      +sign*(M3D::Identity()*(state.template get<mtState::_nor>(ind).getVec().dot(camRor))+state.template get<mtState::_nor>(ind).getVec()*camRor.transpose()))
                  +MPD(qm).matrix()
          )*state.template get<mtState::_nor>(ind).getM();
      J.template block<2,1>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_dep>(ind)) =
          nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)
              *dt*gSM(state.template get<mtState::_nor>(ind).getVec())*camVel;
      J.template block<2,3>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_vel>()) =
          -nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)
              *dt*state.template get<mtState::_dep>(ind)*gSM(state.template get<mtState::_nor>(ind).getVec())*MPD(qVM).matrix();
      J.template block<2,3>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_gyb>()) =
          nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)*(
              sign*(M3D::Identity()-state.template get<mtState::_nor>(ind).getVec()*state.template get<mtState::_nor>(ind).getVec().transpose())
              +state.template get<mtState::_dep>(ind)*gSM(state.template get<mtState::_nor>(ind).getVec())*gSM(qVM.rotate(MrMV))
          )*dt*MPD(qVM).matrix();
      if(doVECalibration){
        J.template block<1,3>(mtState::template getId<mtState::_dep>(ind),mtState::template getId<mtState::_vea>()) =
            dt*pow(state.template get<mtState::_dep>(ind),2)*state.template get<mtState::_nor>(ind).getVec().transpose()*gSM(camVel);
        J.template block<1,3>(mtState::template getId<mtState::_dep>(ind),mtState::template getId<mtState::_vep>()) =
            dt*pow(state.template get<mtState::_dep>(ind),2)*state.template get<mtState::_nor>(ind).getVec().transpose()*MPD(qVM).matrix()*gSM(imuRor);

        J.template block<2,3>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_vea>()) =
            -nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)*(
                sign*(M3D::Identity()-state.template get<mtState::_nor>(ind).getVec()*state.template get<mtState::_nor>(ind).getVec().transpose())
            )*dt*gSM(camRor)
            +nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)
                *dt*state.template get<mtState::_dep>(ind)*gSM(state.template get<mtState::_nor>(ind).getVec())*gSM(camVel);
        J.template block<2,3>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_vep>()) =
            nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)
                *dt*state.template get<mtState::_dep>(ind)*gSM(state.template get<mtState::_nor>(ind).getVec())*MPD(qVM).matrix()*gSM(imuRor);
      }
//      J(mtState::template getId<mtState::_dep>(ind),mtState::template getId<mtState::_dep>(ind)) = 1.0;
//      J.template block<2,2>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_nor>(ind)) = Eigen::Matrix2d::Identity();
    }
    if(doVECalibration){
      J.template block<3,3>(mtState::template getId<mtState::_vep>(),mtState::template getId<mtState::_vep>()) = M3D::Identity();
      J.template block<3,3>(mtState::template getId<mtState::_vea>(),mtState::template getId<mtState::_vea>()) = M3D::Identity();
    }
    return J;
  }
  mtJacNoise jacNoise(const mtState& state, const mtMeas& meas, double dt) const{
    mtJacNoise J;
    V3D dOmega = -dt*(meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    V3D imuRor = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    QPD qVM = qVM_;
    V3D MrMV = MrMV_;
    if(doVECalibration){
      qVM = state.template get<mtState::_vea>();
      MrMV = state.template get<mtState::_vep>();
    }
    V3D camRor = qVM.rotate(imuRor);
    V3D camVel = qVM.rotate(V3D(imuRor.cross(MrMV)-state.template get<mtState::_vel>()));
    J.setZero();
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_pos>())*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_vel>())*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        -MPD(state.template get<mtState::_att>()).matrix()*Lmat(dOmega)*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vep>(),mtNoise::template getId<mtNoise::_vep>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vea>(),mtNoise::template getId<mtNoise::_vea>()) = M3D::Identity()*sqrt(dt);
    NormalVectorElement nOut;
    const FeatureManager<4,8,mtState::nMax_>& fManager = state.template get<mtState::_aux>().fManager_;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      V3D dm = dt*(gSM(state.template get<mtState::_nor>(ind).getVec())*camVel*state.template get<mtState::_dep>(ind)
          - sign*(M3D::Identity()-state.template get<mtState::_nor>(ind).getVec()*state.template get<mtState::_nor>(ind).getVec().transpose())*camRor);
      QPD qm = qm.exponentialMap(dm);
      nOut = state.template get<mtState::_nor>(ind).rotated(qm);
      J(mtState::template getId<mtState::_dep>(ind),mtNoise::template getId<mtNoise::_dep>(ind)) = sqrt(dt);
      J.template block<1,3>(mtState::template getId<mtState::_dep>(ind),mtNoise::template getId<mtNoise::_att>()) =
          -sqrt(dt)*pow(state.template get<mtState::_dep>(ind),2)*state.template get<mtState::_nor>(ind).getVec().transpose()*gSM(qVM.rotate(MrMV))*MPD(qVM).matrix();
      J.template block<2,2>(mtState::template getId<mtState::_nor>(ind),mtNoise::template getId<mtNoise::_nor>(ind)) =
          nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)*state.template get<mtState::_nor>(ind).getN()*sqrt(dt);
      J.template block<2,3>(mtState::template getId<mtState::_nor>(ind),mtNoise::template getId<mtNoise::_att>()) =
          -nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(ind).getVec()))*Lmat(dm)*(
              sign*(M3D::Identity()-state.template get<mtState::_nor>(ind).getVec()*state.template get<mtState::_nor>(ind).getVec().transpose())
              +state.template get<mtState::_dep>(ind)*gSM(state.template get<mtState::_nor>(ind).getVec())*gSM(qVM.rotate(MrMV))
           )*sqrt(dt)*MPD(qVM).matrix();
//      J.template block<2,2>(mtState::template getId<mtState::_nor>(ind),mtNoise::template getId<mtNoise::_nor>(ind)) = Eigen::Matrix2d::Identity()*sqrt(dt);
    }
    return J;
  }
};

}


#endif /* IMUPREDICTION_HPP_ */
