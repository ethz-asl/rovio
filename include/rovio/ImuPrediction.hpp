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

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

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
  const V3D g_; /**<Gravity in inertial frame, always aligned with the z-axis.*/
  double inertialMotionRorTh_; /**<Threshold on the rotational rate for motion detection.*/
  double inertialMotionAccTh_; /**<Threshold on the acceleration for motion detection.*/
  ImuPrediction():g_(0,0,-9.81){
    int ind;
    inertialMotionRorTh_ = 0.1;
    inertialMotionAccTh_ = 0.1;
    doubleRegister_.registerScalar("MotionDetection.inertialMotionRorTh",inertialMotionRorTh_);
    doubleRegister_.registerScalar("MotionDetection.inertialMotionAccTh",inertialMotionAccTh_);
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
  void eval(mtState& output, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt) const{ // TODO: implement without noise for speed
    output.aux().MwWMmeas_ = meas.template get<mtMeas::_gyr>();
    output.aux().MwWMest_  = meas.template get<mtMeas::_gyr>()-state.gyb();
    const V3D imuRor = output.aux().MwWMest_+noise.template get<mtNoise::_att>()/sqrt(dt);
    const V3D dOmega = dt*imuRor;
    QPD dQ = dQ.exponentialMap(-dOmega);
    double d, d_p, p_d, p_d_p;
    LWF::NormalVectorElement tempNormal;
    LWF::NormalVectorElement oldBearing;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      const int camID = state.aux().camID_[i];
      const V3D camRor = state.qCM(camID).rotate(imuRor);
      const V3D camVel = state.qCM(camID).rotate(V3D(imuRor.cross(state.MrMC(camID))-state.MvM()));
      oldBearing = state.CfP(i);
      state.aux().depthMap_.map(state.dep(i),d,d_p,p_d,p_d_p);
      output.dep(i) = state.dep(i)-dt*p_d
          *oldBearing.getVec().transpose()*camVel + noise.template get<mtNoise::_dep>(i)*sqrt(dt);
      V3D dm = dt*(gSM(oldBearing.getVec())*camVel/d
          + (M3D::Identity()-oldBearing.getVec()*oldBearing.getVec().transpose())*camRor)
          + oldBearing.getN()*noise.template get<mtNoise::_nor>(i)*sqrt(dt);
      QPD qm = qm.exponentialMap(dm);
      output.CfP(i) = oldBearing.rotated(qm);
      // WARP corners
      for(unsigned int j=0;j<2;j++){
        oldBearing.boxPlus(state.aux().bearingCorners_[i][j],tempNormal);
        dm = dt*(gSM(tempNormal.getVec())*camVel/d
            + (M3D::Identity()-tempNormal.getVec()*tempNormal.getVec().transpose())*camRor);
        qm = qm.exponentialMap(dm);
        tempNormal = tempNormal.rotated(qm);
        tempNormal.boxMinus(output.CfP(i),output.aux().bearingCorners_[i][j]);
      }
    }
    output.WrWM() = state.WrWM()-dt*(state.qWM().rotate(state.MvM())
        -noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.MvM() = (M3D::Identity()-gSM(dOmega))*state.MvM()
        -dt*(meas.template get<mtMeas::_acc>()-state.acb()+state.qWM().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.acb() = state.acb()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.gyb() = state.gyb()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    output.qWM() = state.qWM()*dQ;
    for(unsigned int i=0;i<mtState::nCam_;i++){
      output.MrMC(i) = state.MrMC(i)+noise.template get<mtNoise::_vep>(i)*sqrt(dt);
      dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>(i)*sqrt(dt));
      output.qCM(i) = dQ*state.qCM(i);
    }
    output.aux().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.fix();
    if(detectInertialMotion(state,meas)){
      output.aux().timeSinceLastInertialMotion_ = 0;
    } else {
      output.aux().timeSinceLastInertialMotion_ = output.aux().timeSinceLastInertialMotion_ + dt;
    }
    output.aux().timeSinceLastImageMotion_ = output.aux().timeSinceLastImageMotion_ + dt;
  }
  void noMeasCase(mtFilterState& filterState, mtMeas& meas, double dt){
    meas.template get<mtMeas::_gyr>() = filterState.state_.gyb();
    meas.template get<mtMeas::_acc>() = filterState.state_.acb()-filterState.state_.qWM().inverseRotate(g_);
  }
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.gyb();
    const V3D dOmega = dt*imuRor;
    double d, d_p, p_d, p_d_p;
    F.setZero();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) = -dt*MPD(state.qWM()).matrix();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_att>()) = -dt*gSM(state.qWM().rotate(state.MvM()));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) = (M3D::Identity()-gSM(dOmega));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) = dt*M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.MvM());
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) = dt*MPD(state.qWM()).matrix().transpose()*gSM(g_);
    F.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) = dt*MPD(state.qWM()).matrix()*Lmat(-dOmega);
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) = M3D::Identity();
    LWF::NormalVectorElement nOut;
    QPD qm;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      const int camID = state.aux().camID_[i];
      const V3D camRor = state.qCM(camID).rotate(imuRor);
      const V3D camVel = state.qCM(camID).rotate(V3D(imuRor.cross(state.MrMC(camID))-state.MvM()));
      state.aux().depthMap_.map(state.dep(i),d,d_p,p_d,p_d_p);
      V3D dm = dt*(gSM(state.CfP(i).getVec())*camVel/d
          + (M3D::Identity()-state.CfP(i).getVec()*state.CfP(i).getVec().transpose())*camRor);
      qm = qm.exponentialMap(dm);
      nOut = state.CfP(i).rotated(qm);
      F(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_dep>(i)) = 1.0 - dt*p_d_p
              *state.CfP(i).getVec().transpose()*camVel;
      F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_vel>()) =
          dt*p_d*state.CfP(i).getVec().transpose()*MPD(state.qCM(camID)).matrix();
      F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_gyb>()) =
          -dt*p_d*state.CfP(i).getVec().transpose()*gSM(state.qCM(camID).rotate(state.MrMC(camID)))*MPD(state.qCM(camID)).matrix();
      F.template block<1,2>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_nor>(i)) =
          -dt*p_d*camVel.transpose()*state.CfP(i).getM();
      F.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_nor>(i)) =
          nOut.getM().transpose()*(
                  dt*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)*(
                      -1.0/d*gSM(camVel)
                      - (M3D::Identity()*(state.CfP(i).getVec().dot(camRor))+state.CfP(i).getVec()*camRor.transpose()))
                  +MPD(qm).matrix()
          )*state.CfP(i).getM();
      F.template block<2,1>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_dep>(i)) =
          -nOut.getM().transpose()*gSM(nOut.getVec())*Lmat(dm)
              *dt*gSM(state.CfP(i).getVec())*camVel*(d_p/(d*d));
      F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_vel>()) =
          -nOut.getM().transpose()*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)
              *dt/d*gSM(state.CfP(i).getVec())*MPD(state.qCM(camID)).matrix();
      F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_gyb>()) =
          nOut.getM().transpose()*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)*(
              - (M3D::Identity()-state.CfP(i).getVec()*state.CfP(i).getVec().transpose())
              +1.0/d*gSM(state.CfP(i).getVec())*gSM(state.qCM(camID).rotate(state.MrMC(camID)))
          )*dt*MPD(state.qCM(camID)).matrix();
      if(state.aux().doVECalibration_){
        F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_vea>(camID)) =
            -dt*p_d*state.CfP(i).getVec().transpose()*gSM(camVel);
        F.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtState::template getId<mtState::_vep>(camID)) =
            -dt*p_d*state.CfP(i).getVec().transpose()*MPD(state.qCM(camID)).matrix()*gSM(imuRor);

        F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_vea>(camID)) =
            nOut.getM().transpose()*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)*(
                (M3D::Identity()-state.CfP(i).getVec()*state.CfP(i).getVec().transpose())
            )*dt*gSM(camRor)
            +nOut.getM().transpose()*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)
                *dt/d*gSM(state.CfP(i).getVec())*gSM(camVel);
        F.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtState::template getId<mtState::_vep>(camID)) =
            nOut.getM().transpose()*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)
                *dt/d*gSM(state.CfP(i).getVec())*MPD(state.qCM(camID)).matrix()*gSM(imuRor);
      }
    }
    for(unsigned int i=0;i<mtState::nCam_;i++){
      F.template block<3,3>(mtState::template getId<mtState::_vep>(i),mtState::template getId<mtState::_vep>(i)) = M3D::Identity();
      F.template block<3,3>(mtState::template getId<mtState::_vea>(i),mtState::template getId<mtState::_vea>(i)) = M3D::Identity();
    }
  }
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.gyb();
    const V3D dOmega = dt*imuRor;
    double d, d_p, p_d, p_d_p;
    G.setZero();
    G.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.MvM())*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        -MPD(state.qWM()).matrix()*Lmat(-dOmega)*sqrt(dt);
    for(unsigned int i=0;i<mtState::nCam_;i++){
      G.template block<3,3>(mtState::template getId<mtState::_vep>(i),mtNoise::template getId<mtNoise::_vep>(i)) = M3D::Identity()*sqrt(dt);
      G.template block<3,3>(mtState::template getId<mtState::_vea>(i),mtNoise::template getId<mtNoise::_vea>(i)) = M3D::Identity()*sqrt(dt);
    }
    LWF::NormalVectorElement nOut;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      const int camID = state.aux().camID_[i];
      const V3D camRor = state.qCM(camID).rotate(imuRor);
      const V3D camVel = state.qCM(camID).rotate(V3D(imuRor.cross(state.MrMC(camID))-state.MvM()));
      state.aux().depthMap_.map(state.dep(i),d,d_p,p_d,p_d_p);
      V3D dm = dt*(gSM(state.CfP(i).getVec())*camVel/d
          + (M3D::Identity()-state.CfP(i).getVec()*state.CfP(i).getVec().transpose())*camRor);
      QPD qm = qm.exponentialMap(dm);
      nOut = state.CfP(i).rotated(qm);
      G(mtState::template getId<mtState::_dep>(i),mtNoise::template getId<mtNoise::_dep>(i)) = sqrt(dt);
      G.template block<1,3>(mtState::template getId<mtState::_dep>(i),mtNoise::template getId<mtNoise::_att>()) =
          sqrt(dt)*p_d*state.CfP(i).getVec().transpose()*gSM(state.qCM(camID).rotate(state.MrMC(camID)))*MPD(state.qCM(camID)).matrix();
      G.template block<2,2>(mtState::template getId<mtState::_nor>(i),mtNoise::template getId<mtNoise::_nor>(i)) =
          nOut.getM().transpose()*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)*state.CfP(i).getN()*sqrt(dt);
      G.template block<2,3>(mtState::template getId<mtState::_nor>(i),mtNoise::template getId<mtNoise::_att>()) =
          -nOut.getM().transpose()*gSM(qm.rotate(state.CfP(i).getVec()))*Lmat(dm)*(
              - (M3D::Identity()-state.CfP(i).getVec()*state.CfP(i).getVec().transpose())
              +1.0/d*gSM(state.CfP(i).getVec())*gSM(state.qCM(camID).rotate(state.MrMC(camID)))
           )*sqrt(dt)*MPD(state.qCM(camID)).matrix();
    }
  }
  bool detectInertialMotion(const mtState& state, const mtMeas& meas) const{
    const V3D imuRor = meas.template get<mtMeas::_gyr>()-state.gyb();
    const V3D imuAcc = meas.template get<mtMeas::_acc>()-state.acb()+state.qWM().inverseRotate(g_);
    return (imuRor.norm() > inertialMotionRorTh_) | (imuAcc.norm() > inertialMotionAccTh_);
  }
};

}


#endif /* ROVIO_IMUPREDICTION_HPP_ */
