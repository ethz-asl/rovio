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

#ifndef ROVIO_LANDMARKOUTPUTIMU_HPP_
#define ROVIO_LANDMARKOUTPUTIMU_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"
#include "rovio/MultiCamera.hpp"

namespace rovio {

class LandmarkOutput: public LWF::State<LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _lmk = 0;
  LandmarkOutput(){
    static_assert(_lmk+1==E_,"Error with indices");
    this->template getName<_lmk>() = "lmk";
  }
  virtual ~LandmarkOutput(){};

  //@{
  /** \brief Get/Set the feature as 3D vector
   *
   *  @return a reference to the landmark
   */
  inline V3D& lmk(){
    return this->template get<_lmk>();
  }
  inline const V3D& lmk() const{
    return this->template get<_lmk>();
  }
  //@}
};

template<typename STATE>
class LandmarkOutputImuCT:public LWF::CoordinateTransform<STATE,LandmarkOutput>{
 public:
  typedef LWF::CoordinateTransform<STATE,LandmarkOutput> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  int ID_;
  MultiCamera<STATE::nCam_>* mpMultiCamera_;
  LandmarkOutputImuCT(MultiCamera<STATE::nCam_>* mpMultiCamera){
    mpMultiCamera_ = mpMultiCamera;
    ID_ = -1;
  };
  virtual ~LandmarkOutputImuCT(){};
  void setFeatureID(int ID){
    ID_ = ID;
  }
  void evalTransform(mtOutput& output, const mtInput& input) const{
    input.updateMultiCameraExtrinsics(mpMultiCamera_);
    // BrBP = BrBC + qCB^T(d_in*nor_in)
    const V3D CrCP = input.dep(ID_).getDistance()*input.CfP(ID_).get_nor().getVec();
    output.template get<mtOutput::_lmk>() = mpMultiCamera_->BrBC_[input.CfP(ID_).camID_] + mpMultiCamera_->qCB_[input.CfP(ID_).camID_].inverseRotate(CrCP);
  }
  void jacTransform(MXD& J, const mtInput& input) const{
    J.setZero();
    input.updateMultiCameraExtrinsics(mpMultiCamera_);
    const V3D CrCP = input.dep(ID_).getDistance()*input.CfP(ID_).get_nor().getVec();
    const Eigen::Matrix<double,3,2> J_CrCP_nor = input.dep(ID_).getDistance()*input.CfP(ID_).get_nor().getM();
    const Eigen::Matrix<double,3,1> J_CrCP_d = input.CfP(ID_).get_nor().getVec()*input.dep(ID_).getDistanceDerivative();
    const M3D mBC = MPD(mpMultiCamera_->qCB_[input.CfP(ID_).camID_].inverted()).matrix();

    J.template block<3,2>(mtOutput::template getId<mtOutput::_lmk>(),mtInput::template getId<mtInput::_fea>(ID_)) = mBC*J_CrCP_nor;
    J.template block<3,1>(mtOutput::template getId<mtOutput::_lmk>(),mtInput::template getId<mtInput::_fea>(ID_)+2) = mBC*J_CrCP_d;

    if(input.aux().doVECalibration_){
      J.template block<3,3>(mtOutput::template getId<mtOutput::_lmk>(),mtInput::template getId<mtInput::_vep>()) = M3D::Identity();
      J.template block<3,3>(mtOutput::template getId<mtOutput::_lmk>(),mtInput::template getId<mtInput::_vea>()) = mBC*gSM(CrCP);
    }
  }
};

}


#endif /* ROVIO_LANDMARKOUTPUTIMU_HPP_ */
