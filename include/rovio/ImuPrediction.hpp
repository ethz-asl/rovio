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

#ifndef ROVIO_IMUPREDICTION_HPP_
#define ROVIO_IMUPREDICTION_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/Prediction.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/Camera.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

class DepthMap{
 public:
  enum DepthType{
   REGULAR,
   INVERSE
  } type_;
  DepthMap(const DepthType& type = REGULAR){
    setType(type);
  }
  void setType(const DepthType& type){
    type_ = type;
  }
  void setType(const int& type){
    switch(type){
      case 0:
        type_ = REGULAR;
        break;
      case 1:
        type_ = INVERSE;
        break;
      default:
        std::cout << "Invalid type for depth parametrization: " << type << std::endl;
        type_ = REGULAR;
        break;
    }
  }
  void map(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    switch(type_){
      case REGULAR:
        mapRegular(p,d,d_p,p_d,p_d_p);
        break;
      case INVERSE:
        mapInverse(p,d,d_p,p_d,p_d_p);
        break;
      default:
        mapRegular(p,d,d_p,p_d,p_d_p);
        break;
    }
  }
  void mapRegular(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    d = p;
    d_p = 1.0;
    p_d = 1.0;
    p_d_p = 0.0;
  }
  void mapInverse(const double& p, double& d, double& d_p, double& p_d, double& p_d_p) const{
    double p_temp = p;
    if(fabs(p_temp) < 1e-6){
      if(p_temp >= 0){
        p_temp = 1e-6;
      } else {
        p_temp = -1e-6;
      }
    }
    d = 1/p_temp;
    d_p = -d*d;
    p_d = -p_temp*p_temp;
    p_d_p = -2*p_temp;
  }
};

template<typename FILTERSTATE>
class ImuPrediction: public LWF::Prediction<FILTERSTATE>{
 public:
  typedef LWF::Prediction<FILTERSTATE> Base;
  using Base::eval;
  using Base::prenoiP_;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::boolRegister_;
  using Base::disablePreAndPostProcessingWarning_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  const V3D g_;
  QPD qVM_;
  V3D MrMV_;
  bool doVECalibration_;
  DepthMap depthMap_; // TODO: move to state
  int depthTypeInt_;
  ImuPrediction():g_(0,0,-9.81){
    qVM_.setIdentity();
    MrMV_.setZero();
    doVECalibration_ = true;
    depthTypeInt_ = 1;
    depthMap_.setType(depthTypeInt_);
    boolRegister_.registerScalar("doVECalibration",doVECalibration_);
    intRegister_.registerScalar("depthType",depthTypeInt_);
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
    int ind;
    for(int i=0;i<mtState::nMax_;i++){
      ind = mtNoise::template getId<mtNoise::_nor>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind,ind));
      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind+1,ind+1));
      ind = mtNoise::template getId<mtNoise::_dep>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
      doubleRegister_.registerScalar("PredictionNoise.dep",prenoiP_(ind,ind));
    }
    disablePreAndPostProcessingWarning_ = true;
  };
  ~ImuPrediction(){};
  void setExtrinsics(const Eigen::Matrix3d& R_VM, const Eigen::Vector3d& VrVM){
    rot::RotationMatrixAD R(R_VM);
    qVM_ = QPD(R.getPassive());
    MrMV_ = -qVM_.inverseRotate(VrVM);
  }
  void refreshProperties(){
    depthMap_.setType(depthTypeInt_);
  };
  void eval(mtState& output, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt) const{ // TODO: implement without noise for speed
    output.template get<mtState::_aux>().MwIMmeas_ = meas.template get<mtMeas::_gyr>();
    output.template get<mtState::_aux>().MwIMest_ = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D imuRor = output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt);
    const V3D dOmega = dt*imuRor;
    QPD dQ = dQ.exponentialMap(-dOmega);
    QPD qVM = qVM_;
    V3D MrMV = MrMV_;
    if(doVECalibration_){
      qVM = state.template get<mtState::_vea>();
      MrMV = state.template get<mtState::_vep>();
    }
    const V3D camRor = qVM.rotate(imuRor);
    const V3D camVel = qVM.rotate(V3D(imuRor.cross(MrMV)-state.template get<mtState::_vel>()));
    double d, d_p, p_d, p_d_p;

    output.template get<mtState::_pos>() = state.template get<mtState::_pos>()-dt*(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>())
        -noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.template get<mtState::_vel>() = (M3D::Identity()-gSM(dOmega))*state.template get<mtState::_vel>()
        -dt*(meas.template get<mtMeas::_acc>()-state.template get<mtState::_acb>()+state.template get<mtState::_att>().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.template get<mtState::_acb>() = state.template get<mtState::_acb>()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.template get<mtState::_gyb>() = state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    output.template get<mtState::_att>() = state.template get<mtState::_att>()*dQ;
    output.template get<mtState::_vep>() = state.template get<mtState::_vep>()+noise.template get<mtNoise::_vep>()*sqrt(dt);
    dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>()*sqrt(dt));
    output.template get<mtState::_vea>() = dQ*state.template get<mtState::_vea>();
    const LWF::NormalVectorElement* mpNormal;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      mpNormal = &state.template get<mtState::_nor>(i);
      depthMap_.map(state.template get<mtState::_dep>(i),d,d_p,p_d,p_d_p);
      output.template get<mtState::_dep>(i) = state.template get<mtState::_dep>(i)-dt*p_d
          *mpNormal->getVec().transpose()*camVel + noise.template get<mtNoise::_dep>(i)*sqrt(dt);
      V3D dm = dt*(gSM(mpNormal->getVec())*camVel/d
          + (M3D::Identity()-mpNormal->getVec()*mpNormal->getVec().transpose())*camRor)
          + mpNormal->getN()*noise.template get<mtNoise::_nor>(i)*sqrt(dt);
      QPD qm = qm.exponentialMap(dm);
      output.template get<mtState::_nor>(i) = mpNormal->rotated(qm);
      // WARP corners
      for(unsigned int j=0;j<2;j++){
        mpNormal = &state.template get<mtState::_aux>().corners_[i][j];
        dm = dt*(gSM(mpNormal->getVec())*camVel/d
            + (M3D::Identity()-mpNormal->getVec()*mpNormal->getVec().transpose())*camRor);
        qm = qm.exponentialMap(dm);
        output.template get<mtState::_aux>().corners_[i][j] = mpNormal->rotated(qm);
      }
    }
    output.template get<mtState::_aux>().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.fix();
  }
  void noMeasCase(mtFilterState& filterState, mtMeas& meas, double dt){
    meas.template get<mtMeas::_gyr>() = filterState.state_.template get<mtState::_gyb>();
    meas.template get<mtMeas::_acc>() = filterState.state_.template get<mtState::_acb>()-filterState.state_.template get<mtState::_att>().inverseRotate(g_);
  }
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D dOmega = dt*imuRor;
    QPD qVM = qVM_;
    V3D MrMV = MrMV_;
    if(doVECalibration_){
      qVM = state.template get<mtState::_vea>();
      MrMV = state.template get<mtState::_vep>();
    }
    const V3D camRor = qVM.rotate(imuRor);
    const V3D camVel = qVM.rotate(V3D(imuRor.cross(MrMV)-state.template get<mtState::_vel>()));
    double d, d_p, p_d, p_d_p;
    F.setZero();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) = -dt*MPD(state.template get<mtState::_att>()).matrix();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_att>()) = -dt*gSM(state.template get<mtState::_att>().rotate(state.template get<mtState::_vel>()));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) = (M3D::Identity()-gSM(dOmega));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) = dt*M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.template get<mtState::_vel>());
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) = dt*MPD(state.template get<mtState::_att>()).matrix().transpose()*gSM(g_);
    F.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) = dt*MPD(state.template get<mtState::_att>()).matrix()*LWF::Lmat(-dOmega);
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) = M3D::Identity();
    LWF::NormalVectorElement nOut;
    QPD qm;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      depthMap_.map(state.template get<mtState::_dep>(i),d,d_p,p_d,p_d_p);
      V3D dm = dt*(gSM(state.template get<mtState::_nor>(i).getVec())*camVel/d
          + (M3D::Identity()-state.template get<mtState::_nor>(i).getVec()*state.template get<mtState::_nor>(i).getVec().transpose())*camRor);
      qm = qm.exponentialMap(dm);
      nOut = state.template get<mtState::_nor>(i).rotated(qm);
      F(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)) = 1.0 - dt*p_d_p
              *state.template get<mtState::_nor>(i).getVec().transpose()*camVel;
      F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_vel>()) =
          dt*p_d*state.template get<mtState::_nor>(i).getVec().transpose()*MPD(qVM).matrix();
      F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_gyb>()) =
          -dt*p_d*state.template get<mtState::_nor>(i).getVec().transpose()*gSM(qVM.rotate(MrMV))*MPD(qVM).matrix();
      F.template block<1,2>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_nor>(i)) =
          -dt*p_d*camVel.transpose()*state.template get<mtState::_nor>(i).getM();
      F.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)) =
          nOut.getM().transpose()*(
                  dt*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)*(
                      -1.0/d*gSM(camVel)
                      - (M3D::Identity()*(state.template get<mtState::_nor>(i).getVec().dot(camRor))+state.template get<mtState::_nor>(i).getVec()*camRor.transpose()))
                  +MPD(qm).matrix()
          )*state.template get<mtState::_nor>(i).getM();
      F.template block<2,1>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_dep>(i)) =
          -nOut.getM().transpose()*gSM(nOut.getVec())*LWF::Lmat(dm)
              *dt*gSM(state.template get<mtState::_nor>(i).getVec())*camVel*(d_p/(d*d));
      F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_vel>()) =
          -nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)
              *dt/d*gSM(state.template get<mtState::_nor>(i).getVec())*MPD(qVM).matrix();
      F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_gyb>()) =
          nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)*(
              - (M3D::Identity()-state.template get<mtState::_nor>(i).getVec()*state.template get<mtState::_nor>(i).getVec().transpose())
              +1.0/d*gSM(state.template get<mtState::_nor>(i).getVec())*gSM(qVM.rotate(MrMV))
          )*dt*MPD(qVM).matrix();
      if(doVECalibration_){
        F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_vea>()) =
            -dt*p_d*state.template get<mtState::_nor>(i).getVec().transpose()*gSM(camVel);
        F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_vep>()) =
            -dt*p_d*state.template get<mtState::_nor>(i).getVec().transpose()*MPD(qVM).matrix()*gSM(imuRor);

        F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_vea>()) =
            nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)*(
                (M3D::Identity()-state.template get<mtState::_nor>(i).getVec()*state.template get<mtState::_nor>(i).getVec().transpose())
            )*dt*gSM(camRor)
            +nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)
                *dt/d*gSM(state.template get<mtState::_nor>(i).getVec())*gSM(camVel);
        F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_vep>()) =
            nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)
                *dt/d*gSM(state.template get<mtState::_nor>(i).getVec())*MPD(qVM).matrix()*gSM(imuRor);
      }
    }
    F.template block<3,3>(mtState::template getId<mtState::_vep>(),mtState::template getId<mtState::_vep>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vea>(),mtState::template getId<mtState::_vea>()) = M3D::Identity();
  }
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    const V3D dOmega = dt*imuRor;
    QPD qVM = qVM_;
    V3D MrMV = MrMV_;
    if(doVECalibration_){
      qVM = state.template get<mtState::_vea>();
      MrMV = state.template get<mtState::_vep>();
    }
    const V3D camRor = qVM.rotate(imuRor);
    const V3D camVel = qVM.rotate(V3D(imuRor.cross(MrMV)-state.template get<mtState::_vel>()));
    double d, d_p, p_d, p_d_p;
    G.setZero();
    G.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_vel>())*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        -MPD(state.template get<mtState::_att>()).matrix()*LWF::Lmat(-dOmega)*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vep>(),mtNoise::template getId<mtNoise::_vep>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vea>(),mtNoise::template getId<mtNoise::_vea>()) = M3D::Identity()*sqrt(dt);
    LWF::NormalVectorElement nOut;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      depthMap_.map(state.template get<mtState::_dep>(i),d,d_p,p_d,p_d_p);
      V3D dm = dt*(gSM(state.template get<mtState::_nor>(i).getVec())*camVel/d
          + (M3D::Identity()-state.template get<mtState::_nor>(i).getVec()*state.template get<mtState::_nor>(i).getVec().transpose())*camRor);
      QPD qm = qm.exponentialMap(dm);
      nOut = state.template get<mtState::_nor>(i).rotated(qm);
      G(mtState::template getId<mtState::_dep>(i),mtNoise::template getId<mtNoise::_dep>(i)) = sqrt(dt);
      G.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtNoise::template getId<mtNoise::_att>()) =
          sqrt(dt)*p_d*state.template get<mtState::_nor>(i).getVec().transpose()*gSM(qVM.rotate(MrMV))*MPD(qVM).matrix();
      G.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtNoise::template getId<mtNoise::_nor>(i)) =
          nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)*state.template get<mtState::_nor>(i).getN()*sqrt(dt);
      G.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtNoise::template getId<mtNoise::_att>()) =
          -nOut.getM().transpose()*gSM(qm.rotate(state.template get<mtState::_nor>(i).getVec()))*LWF::Lmat(dm)*(
              - (M3D::Identity()-state.template get<mtState::_nor>(i).getVec()*state.template get<mtState::_nor>(i).getVec().transpose())
              +1.0/d*gSM(state.template get<mtState::_nor>(i).getVec())*gSM(qVM.rotate(MrMV))
           )*sqrt(dt)*MPD(qVM).matrix();
    }
  }
};

}


#endif /* ROVIO_IMUPREDICTION_HPP_ */
