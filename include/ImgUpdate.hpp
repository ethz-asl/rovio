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
#define IMGUPDATE_HPP_ // TODO

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "Update.hpp"
#include "State.hpp"
#include "FilterState.hpp"
#include <cv_bridge/cv_bridge.h>

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
      ID_[i] = 0;
    }
  };
  ~ImgUpdateMeasAuxiliary(){};
  cv::Mat img_;
  unsigned int ID_[nMax];
};
template<typename STATE>
class ImgUpdateMeas: public State<ArrayElement<NormalVectorElement,STATE::nMax_>,ImgUpdateMeasAuxiliary<STATE::nMax_>>{
 public:
  typedef State<ArrayElement<NormalVectorElement,STATE::nMax_>,ImgUpdateMeasAuxiliary<STATE::nMax_>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  static constexpr unsigned int _aux = _nor+1;
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
  typedef typename Base::mtState mtState;
  typedef typename Base::mtCovMat mtCovMat;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  ImgUpdate(){
    qVM_.setIdentity();
    MrMV_.setZero();
    initCovFeature_.setIdentity();
    initDepth_ = 0;
    doubleRegister_.registerScaledUnitMatrix("initCovFeature",initCovFeature_);
    doubleRegister_.registerScalar("initDepth",initDepth_);
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
  };
  ~ImgUpdate(){};
  rot::RotationQuaternionPD qVM_;
  Eigen::Vector3d MrMV_;
  Eigen::Matrix3d initCovFeature_;
  double initDepth_;
  mtInnovation eval(const mtState& state, const mtMeas& meas, const mtNoise noise, double dt = 0.0) const{
    mtInnovation y;
//    /* Bearing vector error
//     * 0 = m - m_meas + n
//     */
    NormalVectorElement m_est, m_meas;
    unsigned int stateInd, ID;
    for(unsigned int i=0;i<STATE::nMax_;i++){
      ID = meas.template get<mtMeas::_aux>().ID_[i];
      if(ID != 0){
        stateInd = state.template get<mtState::_aux>().indFeature_.at(ID);
        m_est.n_ = state.template get<mtState::_nor>(stateInd);
        m_meas.n_ = meas.template get<mtMeas::_nor>(i);
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
    unsigned int stateInd, ID;
    Eigen::Vector3d vec;
    double vecNorm,a; // todo
    for(unsigned int i=0;i<STATE::nMax_;i++){
      ID = meas.template get<mtMeas::_aux>().ID_[i];
      if(ID != 0){
        stateInd = state.template get<mtState::_aux>().indFeature_.at(ID);
        m_est.n_ = state.template get<mtState::_nor>(stateInd);
        m_meas.n_ = meas.template get<mtMeas::_nor>(i);
        rot::RotationQuaternionPD q;
        q.setFromVectors(m_meas.n_,m_est.n_);
        vec = -q.logarithmicMap(); // vec*std::asin(vecNorm)/vecNorm
        a = vec.norm(); // std::asin(vecNorm)
//        std::cout << -q.logarithmicMap() << std::endl;
//        vec = -m_meas.n_.cross(m_est.n_);
//        vecNorm = vec.norm();
//        std::cout << vec*std::asin(vecNorm)/vecNorm << std::endl;
        J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(i),mtState::template getId<mtState::_nor>(stateInd)) =
            -m_meas.getN().transpose()*(
                Eigen::Matrix3d::Identity()*a/std::sin(a)
                +(vec*vec.transpose())*(1/(std::sqrt(1-std::pow(std::sin(a),2))*pow(a,2))-1/std::sin(a)/a) // TODO: handle special cases
            )*kindr::linear_algebra::getSkewMatrixFromVector(m_meas.n_)*m_est.getM();
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
    unsigned int stateInd, ID;
    for(unsigned int i=0;i<STATE::nMax_;i++){
      ID = meas.template get<mtMeas::_aux>().ID_[i];
      if(ID != 0){
        if(state.template get<mtState::_aux>().indFeature_.count(ID)==0){
          state.template get<mtState::_aux>().addIndex(ID);
          stateInd = state.template get<mtState::_aux>().indFeature_.at(ID);
          state.initializeFeature(cov,stateInd,meas.template get<mtMeas::_nor>(i),initDepth_,initCovFeature_);
        }
      }
    }
  };
};

}


#endif /* IMGUPDATE_HPP_ */
