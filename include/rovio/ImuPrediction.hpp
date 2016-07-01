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

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Prediction.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"

namespace rovio {

template<typename FILTERSTATE>
class ImuPrediction: public LWF::Prediction<FILTERSTATE>{
 public:
  typedef LWF::Prediction<FILTERSTATE> Base;
  using Base::meas_;
  using Base::prenoiP_;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::boolRegister_;
  using Base::disablePreAndPostProcessingWarning_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  const V3D g_; /**<Gravity in inertial frame, always aligned with the z-axis.*/
  double inertialMotionRorTh_; /**<Threshold on the rotational rate for motion detection.*/
  double inertialMotionAccTh_; /**<Threshold on the acceleration for motion detection.*/
  mutable FeatureCoordinates oldC_;
  mutable FeatureDistance oldD_;
  mutable Eigen::Matrix2d bearingVectorJac_;
  ImuPrediction():g_(0,0,-9.81){
    int ind;
    inertialMotionRorTh_ = 0.1;
    inertialMotionAccTh_ = 0.1;
    doubleRegister_.registerScalar("MotionDetection.inertialMotionRorTh",inertialMotionRorTh_);
    doubleRegister_.registerScalar("MotionDetection.inertialMotionAccTh",inertialMotionAccTh_);
    for(int i=0;i<mtState::nMax_;i++){
      ind = mtNoise::template getId<mtNoise::_fea>(i);
      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
      doubleRegister_.removeScalarByVar(prenoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind,ind));
      doubleRegister_.registerScalar("PredictionNoise.nor",prenoiP_(ind+1,ind+1));
      ind = mtNoise::template getId<mtNoise::_fea>(i)+2;
      doubleRegister_.removeScalarByVar(prenoiP_(ind,ind));
      doubleRegister_.registerScalar("PredictionNoise.dep",prenoiP_(ind,ind));
    }
    for(int camID=0;camID<mtState::nCam_;camID++){
      for(int j=0;j<3;j++){
        doubleRegister_.removeScalarByVar(prenoiP_(mtNoise::template getId<mtNoise::_vep>(camID)+j,mtNoise::template getId<mtNoise::_vep>(camID)+j));
        doubleRegister_.removeScalarByVar(prenoiP_(mtNoise::template getId<mtNoise::_vea>(camID)+j,mtNoise::template getId<mtNoise::_vea>(camID)+j));
        doubleRegister_.registerScalar("PredictionNoise.vep",prenoiP_(mtNoise::template getId<mtNoise::_vep>(camID)+j,mtNoise::template getId<mtNoise::_vep>(camID)+j));
        doubleRegister_.registerScalar("PredictionNoise.vea",prenoiP_(mtNoise::template getId<mtNoise::_vea>(camID)+j,mtNoise::template getId<mtNoise::_vea>(camID)+j));
      }
    }
    for(int i=0;i<mtState::nPose_;i++){
      for(int j=0;j<3;j++){
        doubleRegister_.removeScalarByVar(prenoiP_(mtNoise::template getId<mtNoise::_pop>(i)+j,mtNoise::template getId<mtNoise::_pop>(i)+j));
        doubleRegister_.removeScalarByVar(prenoiP_(mtNoise::template getId<mtNoise::_poa>(i)+j,mtNoise::template getId<mtNoise::_poa>(i)+j));
      }
    }
    disablePreAndPostProcessingWarning_ = true;
  };

  /** \brief Destructor
   */
  virtual ~ImuPrediction(){};

  /* /brief Evaluation of prediction
   *
   * @todo implement without noise for speed
   */
  void evalPrediction(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    output.aux().MwWMmeas_ = meas_.template get<mtMeas::_gyr>();
    output.aux().MwWMest_  = meas_.template get<mtMeas::_gyr>()-state.gyb();
    const V3D imuRor = output.aux().MwWMest_+noise.template get<mtNoise::_att>()/sqrt(dt);
    const V3D dOmega = dt*imuRor;
    QPD dQ = dQ.exponentialMap(dOmega);
    for(unsigned int i=0;i<mtState::nMax_;i++){
      const int camID = state.CfP(i).camID_;
      if(&output != &state){
        output.CfP(i) = state.CfP(i);
        output.dep(i) = state.dep(i);
      }
      if(camID >= 0 && camID < mtState::nCam_){
        const V3D camRor = state.qCM(camID).rotate(imuRor);
        const V3D camVel = state.qCM(camID).rotate(V3D(imuRor.cross(state.MrMC(camID))-state.MvM()));
        oldC_ = state.CfP(i);
        oldD_ = state.dep(i);
        output.dep(i).p_ = oldD_.p_-dt*oldD_.getParameterDerivative()*oldC_.get_nor().getVec().transpose()*camVel + noise.template get<mtNoise::_fea>(i)(2)*sqrt(dt);
        V3D dm = -dt*(gSM(oldC_.get_nor().getVec())*camVel/oldD_.getDistance()
            + (M3D::Identity()-oldC_.get_nor().getVec()*oldC_.get_nor().getVec().transpose())*camRor)
            + oldC_.get_nor().getN()*noise.template get<mtNoise::_fea>(i).template block<2,1>(0,0)*sqrt(dt);
        QPD qm = qm.exponentialMap(dm);
        output.CfP(i).set_nor(oldC_.get_nor().rotated(qm));
        // WARP corners
        if(state.CfP(i).trackWarping_){
          bearingVectorJac_ = output.CfP(i).get_nor().getM().transpose()*(dt*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)*(
                                  -1.0/oldD_.getDistance()*gSM(camVel)
                                  - (M3D::Identity()*(oldC_.get_nor().getVec().dot(camRor))+oldC_.get_nor().getVec()*camRor.transpose()))
                              +MPD(qm).matrix())*oldC_.get_nor().getM();
          output.CfP(i).set_warp_nor(bearingVectorJac_*oldC_.get_warp_nor());
        }
      }
    }
    output.WrWM() = state.WrWM()-dt*(state.qWM().rotate(state.MvM())-noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.MvM() = (M3D::Identity()-gSM(dOmega))*state.MvM()-dt*(meas_.template get<mtMeas::_acc>()-state.acb()+state.qWM().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.acb() = state.acb()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.gyb() = state.gyb()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    output.qWM() = state.qWM()*dQ;
    for(unsigned int i=0;i<mtState::nCam_;i++){
      output.MrMC(i) = state.MrMC(i)+noise.template get<mtNoise::_vep>(i)*sqrt(dt);
      dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>(i)*sqrt(dt));
      output.qCM(i) = dQ*state.qCM(i);
    }
    for(unsigned int i=0;i<mtState::nPose_;i++){
      output.poseLin(i) = state.poseLin(i)+noise.template get<mtNoise::_pop>(i)*sqrt(dt);
      dQ = dQ.exponentialMap(noise.template get<mtNoise::_poa>(i)*sqrt(dt));
      output.poseRot(i) = dQ*state.poseRot(i);
    }
    output.aux().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())/dt;
    output.fix();
    if(detectInertialMotion(state,meas_)){
      output.aux().timeSinceLastInertialMotion_ = 0;
    } else {
      output.aux().timeSinceLastInertialMotion_ = output.aux().timeSinceLastInertialMotion_ + dt;
    }
    output.aux().timeSinceLastImageMotion_ = output.aux().timeSinceLastImageMotion_ + dt;
  }
  void noMeasCase(mtFilterState& filterState, mtMeas& meas_, double dt){
    meas_.template get<mtMeas::_gyr>() = filterState.state_.gyb();
    meas_.template get<mtMeas::_acc>() = filterState.state_.acb()-filterState.state_.qWM().inverseRotate(g_);
  }
  void jacPreviousState(MXD& F, const mtState& state, double dt) const{
    const V3D imuRor = meas_.template get<mtMeas::_gyr>()-state.gyb();
    const V3D dOmega = dt*imuRor;
    F.setZero();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) = -dt*MPD(state.qWM()).matrix();
    F.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_att>()) = dt*gSM(state.qWM().rotate(state.MvM()));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) = (M3D::Identity()-gSM(dOmega));
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) = dt*M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) = -dt*gSM(state.MvM());
    F.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) = -dt*MPD(state.qWM()).matrix().transpose()*gSM(g_);
    F.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity();
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) = -dt*MPD(state.qWM()).matrix()*Lmat(dOmega);
    F.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) = M3D::Identity();
    LWF::NormalVectorElement nOut;
    QPD qm;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      const int camID = state.CfP(i).camID_;
      if(camID >= 0 && camID < mtState::nCam_){
        const V3D camRor = state.qCM(camID).rotate(imuRor);
        const V3D camVel = state.qCM(camID).rotate(V3D(imuRor.cross(state.MrMC(camID))-state.MvM()));
        oldC_ = state.CfP(i);
        oldD_ = state.dep(i);
        V3D dm = -dt*(gSM(oldC_.get_nor().getVec())*camVel/oldD_.getDistance()
            + (M3D::Identity()-oldC_.get_nor().getVec()*oldC_.get_nor().getVec().transpose())*camRor);
        qm = qm.exponentialMap(dm);
        nOut = oldC_.get_nor().rotated(qm);
        F(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)+2) = 1.0 - dt*oldD_.getParameterDerivativeCombined()
                *oldC_.get_nor().getVec().transpose()*camVel;
        F.template block<1,3>(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_vel>()) =
            dt*oldD_.getParameterDerivative()*oldC_.get_nor().getVec().transpose()*MPD(state.qCM(camID)).matrix();
        F.template block<1,3>(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_gyb>()) =
            -dt*oldD_.getParameterDerivative()*oldC_.get_nor().getVec().transpose()*gSM(state.qCM(camID).rotate(state.MrMC(camID)))*MPD(state.qCM(camID)).matrix();
        F.template block<1,2>(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_fea>(i)) =
            -dt*oldD_.getParameterDerivative()*camVel.transpose()*oldC_.get_nor().getM();
        F.template block<2,2>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_fea>(i)) =
            nOut.getM().transpose()*(
                    dt*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)*(
                        -1.0/oldD_.getDistance()*gSM(camVel)
                        - (M3D::Identity()*(oldC_.get_nor().getVec().dot(camRor))+oldC_.get_nor().getVec()*camRor.transpose()))
                    +MPD(qm).matrix()
            )*oldC_.get_nor().getM();
        F.template block<2,1>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_fea>(i)+2) =
            -nOut.getM().transpose()*gSM(nOut.getVec())*Lmat(dm)
                *dt*gSM(oldC_.get_nor().getVec())*camVel*(oldD_.getDistanceDerivative()/(oldD_.getDistance()*oldD_.getDistance()));
        F.template block<2,3>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_vel>()) =
            -nOut.getM().transpose()*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)
                *dt/oldD_.getDistance()*gSM(oldC_.get_nor().getVec())*MPD(state.qCM(camID)).matrix();
        F.template block<2,3>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_gyb>()) =
            nOut.getM().transpose()*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)*(
                - (M3D::Identity()-oldC_.get_nor().getVec()*oldC_.get_nor().getVec().transpose())
                +1.0/oldD_.getDistance()*gSM(oldC_.get_nor().getVec())*gSM(state.qCM(camID).rotate(state.MrMC(camID)))
            )*dt*MPD(state.qCM(camID)).matrix();
        if(state.aux().doVECalibration_){
          F.template block<1,3>(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_vea>(camID)) =
              dt*oldD_.getParameterDerivative()*oldC_.get_nor().getVec().transpose()*gSM(camVel);
          F.template block<1,3>(mtState::template getId<mtState::_fea>(i)+2,mtState::template getId<mtState::_vep>(camID)) =
              -dt*oldD_.getParameterDerivative()*oldC_.get_nor().getVec().transpose()*MPD(state.qCM(camID)).matrix()*gSM(imuRor);

          F.template block<2,3>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_vea>(camID)) =
              -nOut.getM().transpose()*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)*(
                  (M3D::Identity()-oldC_.get_nor().getVec()*oldC_.get_nor().getVec().transpose())
              )*dt*gSM(camRor)
              -nOut.getM().transpose()*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)
                  *dt/oldD_.getDistance()*gSM(oldC_.get_nor().getVec())*gSM(camVel);
          F.template block<2,3>(mtState::template getId<mtState::_fea>(i),mtState::template getId<mtState::_vep>(camID)) =
              nOut.getM().transpose()*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)
                  *dt/oldD_.getDistance()*gSM(oldC_.get_nor().getVec())*MPD(state.qCM(camID)).matrix()*gSM(imuRor);
        }
      }
    }
    for(unsigned int i=0;i<mtState::nCam_;i++){
      F.template block<3,3>(mtState::template getId<mtState::_vep>(i),mtState::template getId<mtState::_vep>(i)) = M3D::Identity();
      F.template block<3,3>(mtState::template getId<mtState::_vea>(i),mtState::template getId<mtState::_vea>(i)) = M3D::Identity();
    }
    for(unsigned int i=0;i<mtState::nPose_;i++){
      F.template block<3,3>(mtState::template getId<mtState::_pop>(i),mtState::template getId<mtState::_pop>(i)) = M3D::Identity();
      F.template block<3,3>(mtState::template getId<mtState::_poa>(i),mtState::template getId<mtState::_poa>(i)) = M3D::Identity();
    }
  }
  void jacNoise(MXD& G, const mtState& state, double dt) const{
    const V3D imuRor = meas_.template get<mtMeas::_gyr>()-state.gyb();
    const V3D dOmega = dt*imuRor;
    G.setZero();
    G.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) = gSM(state.MvM())*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()*sqrt(dt);
    G.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) = MPD(state.qWM()).matrix()*Lmat(dOmega)*sqrt(dt);
    for(unsigned int i=0;i<mtState::nCam_;i++){
      G.template block<3,3>(mtState::template getId<mtState::_vep>(i),mtNoise::template getId<mtNoise::_vep>(i)) = M3D::Identity()*sqrt(dt);
      G.template block<3,3>(mtState::template getId<mtState::_vea>(i),mtNoise::template getId<mtNoise::_vea>(i)) = M3D::Identity()*sqrt(dt);
    }
    for(unsigned int i=0;i<mtState::nPose_;i++){
      G.template block<3,3>(mtState::template getId<mtState::_pop>(i),mtNoise::template getId<mtNoise::_pop>(i)) = M3D::Identity()*sqrt(dt);
      G.template block<3,3>(mtState::template getId<mtState::_poa>(i),mtNoise::template getId<mtNoise::_poa>(i)) = M3D::Identity()*sqrt(dt);
    }
    LWF::NormalVectorElement nOut;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      const int camID = state.CfP(i).camID_;
      if(camID >= 0 && camID < mtState::nCam_){
        oldC_ = state.CfP(i);
        oldD_ = state.dep(i);
        const V3D camRor = state.qCM(camID).rotate(imuRor);
        const V3D camVel = state.qCM(camID).rotate(V3D(imuRor.cross(state.MrMC(camID))-state.MvM()));
        V3D dm = -dt*(gSM(oldC_.get_nor().getVec())*camVel/oldD_.getDistance()
            + (M3D::Identity()-oldC_.get_nor().getVec()*oldC_.get_nor().getVec().transpose())*camRor);
        QPD qm = qm.exponentialMap(dm);
        nOut = oldC_.get_nor().rotated(qm);
        G(mtState::template getId<mtState::_fea>(i)+2,mtNoise::template getId<mtNoise::_fea>(i)+2) = sqrt(dt);
        G.template block<1,3>(mtState::template getId<mtState::_fea>(i)+2,mtNoise::template getId<mtNoise::_att>()) =
            sqrt(dt)*oldD_.getParameterDerivative()*oldC_.get_nor().getVec().transpose()*gSM(state.qCM(camID).rotate(state.MrMC(camID)))*MPD(state.qCM(camID)).matrix();
        G.template block<2,2>(mtState::template getId<mtState::_fea>(i),mtNoise::template getId<mtNoise::_fea>(i)) =
            -nOut.getM().transpose()*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)*oldC_.get_nor().getN()*sqrt(dt);
        G.template block<2,3>(mtState::template getId<mtState::_fea>(i),mtNoise::template getId<mtNoise::_att>()) =
            -nOut.getM().transpose()*gSM(qm.rotate(oldC_.get_nor().getVec()))*Lmat(dm)*(
                - (M3D::Identity()-oldC_.get_nor().getVec()*oldC_.get_nor().getVec().transpose())
                +1.0/oldD_.getDistance()*gSM(oldC_.get_nor().getVec())*gSM(state.qCM(camID).rotate(state.MrMC(camID)))
             )*sqrt(dt)*MPD(state.qCM(camID)).matrix();
      }
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
