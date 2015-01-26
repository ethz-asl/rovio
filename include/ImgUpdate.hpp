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

#ifndef IMGUPDATE_HPP_
#define IMGUPDATE_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "Update.hpp"
#include "State.hpp"
#include "FilterState.hpp"
#include <cv_bridge/cv_bridge.h>
#include "common_vision.hpp"
#include <ros-camera.h>

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

using namespace LWF;

template<typename STATE>
class ImgInnovation: public State<ArrayElement<VectorElement<2>,STATE::nMax_>>{
 public:
  typedef State<ArrayElement<VectorElement<2>,STATE::nMax_>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgInnovation(){
    static_assert(_nor+1==E_,"Error with indices");
  };
  ~ImgInnovation(){};
};
template<unsigned int nMax>
class ImgUpdateMeasAuxiliary: public LWF::AuxiliaryBase<ImgUpdateMeasAuxiliary<nMax>>{
 public:
  ImgUpdateMeasAuxiliary(){
    for(unsigned int i=0;i<nMax;i++){
      imgTime_ = 0.0;
    }
  };
  ~ImgUpdateMeasAuxiliary(){};
  cv::Mat img_;
  double imgTime_;
};
template<typename STATE>
class ImgUpdateMeas: public State<ImgUpdateMeasAuxiliary<STATE::nMax_>>{
 public:
  typedef State<ImgUpdateMeasAuxiliary<STATE::nMax_>> Base;
  using Base::E_;
  static constexpr unsigned int _aux = 0;
  ImgUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  ~ImgUpdateMeas(){};
};
template<typename STATE>
class ImgUpdateNoise: public State<ArrayElement<VectorElement<2>,STATE::nMax_>>{
 public:
  typedef State<ArrayElement<VectorElement<2>,STATE::nMax_>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgUpdateNoise(){
    static_assert(_nor+1==E_,"Error with indices");
  };
  ~ImgUpdateNoise(){};
};
template<typename STATE>
class ImgOutlierDetection: public OutlierDetection<ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_nor>(),2,STATE::nMax_>>{
};

template<typename STATE>
class ImgUpdate: public Update<ImgInnovation<STATE>,STATE,ImgUpdateMeas<STATE>,ImgUpdateNoise<STATE>,
    ImgOutlierDetection<STATE>,DummyPrediction,false>{
 public:
  typedef Update<ImgInnovation<STATE>,STATE,ImgUpdateMeas<STATE>,ImgUpdateNoise<STATE>,
      ImgOutlierDetection<STATE>,DummyPrediction,false> Base;
  using typename Base::eval;
  using Base::doubleRegister_;
  using Base::intRegister_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtCovMat mtCovMat;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  ImgUpdate(){
    camera_.reset(new RosCamera("fpga21_cam0.yaml"));
    qVM_.setIdentity();
    MrMV_.setZero();
    initCovFeature_.setIdentity();
    initDepth_ = 0;
    timeForRemoval_ = 10.0;
    minFeatureForDetection_ = 10;
    doubleRegister_.registerScaledUnitMatrix("initCovFeature",initCovFeature_);
    doubleRegister_.registerScalar("initDepth",initDepth_);
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
    doubleRegister_.registerScalar("timeForRemoval",timeForRemoval_);
    intRegister_.registerScalar("minFeatureForDetection",minFeatureForDetection_);
  };
  ~ImgUpdate(){};
  rot::RotationQuaternionPD qVM_;
  Eigen::Vector3d MrMV_;
  Eigen::Matrix3d initCovFeature_;
  double initDepth_;
  double timeForRemoval_;
  int minFeatureForDetection_;
  std::shared_ptr<RosCamera> camera_;
  mtInnovation eval(const mtState& state, const mtMeas& meas, const mtNoise noise, double dt = 0.0) const{
    mtInnovation y;
//    /* Bearing vector error
//     * 0 = m - m_meas + n
//     */
    NormalVectorElement m_est, m_meas;
    for(unsigned int i=0;i<STATE::nMax_;i++){
      if(state.template get<mtState::_aux>().ID_[i] != 0 && state.template get<mtState::_aux>().isVisible_[i]){
        m_est.n_ = state.template get<mtState::_nor>(i);
        m_meas.n_ = state.template get<mtState::_aux>().norInCurrentFrame_[i];
        m_est.boxMinus(m_meas,y.template get<mtInnovation::_nor>(i));
        y.template get<mtInnovation::_nor>(i) += noise.template get<mtNoise::_nor>(i);
      } else {
        y.template get<mtInnovation::_nor>(i) = noise.template get<mtNoise::_nor>(i);
      }
    }
    return y;
  }
  mtJacInput jacInput(const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    mtJacInput J;
    J.setZero();
    NormalVectorElement m_est, m_meas;
    Eigen::Vector3d vec;
    double a, vecNorm, c;
    for(unsigned int i=0;i<STATE::nMax_;i++){
      if(state.template get<mtState::_aux>().ID_[i] != 0 && state.template get<mtState::_aux>().isVisible_[i]){
        m_est.n_ = state.template get<mtState::_nor>(i);
        m_meas.n_ = state.template get<mtState::_aux>().norInCurrentFrame_[i];
        vec = -m_meas.n_.cross(m_est.n_);
        vecNorm = vec.norm();
        c = m_meas.n_.dot(m_est.n_);
        a = std::acos(c);
        if(vecNorm<1e-6){
          if(c>0){
            J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(i),mtState::template getId<mtState::_nor>(i)) =
                -m_meas.getN().transpose()*kindr::linear_algebra::getSkewMatrixFromVector(m_meas.n_)*m_est.getM();
          } else { // TODO: imprecise
            J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(i),mtState::template getId<mtState::_nor>(i)).setZero();
          }
        } else {
          J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(i),mtState::template getId<mtState::_nor>(i)) =
              m_meas.getN().transpose()*(
                  -(Eigen::Matrix3d::Identity()*a/vecNorm - (vec*vec.transpose())*a/pow(vecNorm,3))*kindr::linear_algebra::getSkewMatrixFromVector(m_meas.n_)
                  -(vec*m_meas.n_.transpose())/(std::sqrt(1-std::pow(c,2))*vecNorm)
              )*m_est.getM();
        }
      }
    }
    return J;
  }
  mtJacNoise jacNoise(const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    mtJacNoise J;
    J.setZero();
    for(unsigned int i=0;i<STATE::nMax_;i++){
      J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(i),mtNoise::template getId<mtNoise::_nor>(i)) = Eigen::Matrix2d::Identity();
    }
    return J;
  }
  void preProcess(mtState& state, mtCovMat& cov, const mtMeas& meas){
    state.template get<mtState::_aux>().resetVisible();
    if(meas.template get<mtMeas::_aux>().img_.empty()){
      std::cout << "Img Update Error: Image is empty" << std::endl;
      return;
    } else {
      // Search visible features
      unsigned int ID;
      Eigen::Vector2d vec2;
      Eigen::Vector3d vec3;
      const Eigen::Vector3d e3(0,0,1);
      bool converged;
      uint8_t patch[64] __attribute__ ((aligned (16)));
      for(unsigned int i=0;i<STATE::nMax_;i++){
        ID = state.template get<mtState::_aux>().ID_[i];
        vec3 = state.template get<mtState::_nor>(i);
        if(ID != 0 && e3.dot(vec3)>0){ // check if on front side of camera
          vec2 = camera_->worldToCam(vec3);
          createPatchFromPatchWithBorder(state.template get<mtState::_aux>().patchesWithBorder_[i].data_,patch);
          // TODO: make multiple samples depending on covariance
          converged = align2D(meas.template get<mtMeas::_aux>().img_,state.template get<mtState::_aux>().patchesWithBorder_[i].data_,patch,10,vec2);
          if(converged){
            state.template get<mtState::_aux>().isVisible_[i] = true;
            vec3 = camera_->camToWorld(vec2.x(),vec2.y());
            vec3.normalize();
            state.template get<mtState::_aux>().norInCurrentFrame_[i] = vec3;
          }
        }
      }
    }
  };
  void postProcess(mtState& state, mtCovMat& cov, const mtMeas& meas, mtOutlierDetection* mpOutlierDetection){
    // Check if new feature should be added to the state
    if(meas.template get<mtMeas::_aux>().img_.empty()){
      std::cout << "Img Update Error: Image is empty" << std::endl;
      return;
    } else {
      state.template get<mtState::_aux>().img_ = meas.template get<mtMeas::_aux>().img_;
      state.template get<mtState::_aux>().imgTime_ = meas.template get<mtMeas::_aux>().imgTime_;
      std::vector<cv::Point2f> points_current;
      Eigen::Vector2d vec2;
      for(auto it = state.template get<mtState::_aux>().indFeature_.begin();it != state.template get<mtState::_aux>().indFeature_.end();++it){
        vec2 = camera_->worldToCam(state.template get<mtState::_nor>(it->second));
        points_current.emplace_back(vec2(0),vec2(1));
      }
      std::vector<cv::Point2f> points_temp;
      const int missingFeatureCount = STATE::nMax_-state.template get<mtState::_aux>().getTotFeatureNo();
      if(missingFeatureCount > minFeatureForDetection_){
        DetectFastCorners(state.template get<mtState::_aux>().img_, points_temp, points_current,missingFeatureCount);
      }
      Eigen::Vector3d vec3;
      unsigned int stateInd, ID;
      for(auto it = points_temp.begin();it != points_temp.end();++it){
        vec3 = camera_->camToWorld(it->x,it->y);
        vec3.normalize();
        ID = state.template get<mtState::_aux>().maxID_;
        stateInd = state.template get<mtState::_aux>().addID(ID);
        if(getPatchInterpolated(state.template get<mtState::_aux>().img_,Eigen::Vector2d(it->x,it->y),5,state.template get<mtState::_aux>().patchesWithBorder_[stateInd].data_)){
          state.initializeFeatureState(cov,stateInd,vec3,initDepth_,initCovFeature_);
        } else {
          state.template get<mtState::_aux>().removeID(ID);
        }
      }
      for(unsigned int i=0;i<STATE::nMax_;i++){
        ID = state.template get<mtState::_aux>().ID_[i];
        if(ID != 0 && state.template get<mtState::_aux>().timeSinceVisible_[i] > timeForRemoval_){
          state.template get<mtState::_aux>().removeID(ID);
        }
      }

      // Get newest patch
      for(unsigned int i=0;i<STATE::nMax_;i++){
        ID = state.template get<mtState::_aux>().ID_[i];
        if(ID != 0){
          vec2 = camera_->worldToCam(state.template get<mtState::_nor>(i));
          getPatchInterpolated(state.template get<mtState::_aux>().img_,vec2,5,state.template get<mtState::_aux>().patchesWithBorder_[i].data_);
        }
      }
    }
  };
};

}


#endif /* IMGUPDATE_HPP_ */
