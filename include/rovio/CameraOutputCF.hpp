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

#ifndef ROVIO_CAMERAOUTPUT_HPP_
#define ROVIO_CAMERAOUTPUT_HPP_

#include "rovio/FilterStates.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

namespace rovio {

class StandardOutput: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,3>,LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;         /**<Idx. Position Vector WrWC: Pointing from the World-Frame to the Camera-Frame, expressed in World-Coordinates.*/
  static constexpr unsigned int _vel = _pos+1;    /**<Idx. Velocity Vector  CvC: Absolute velocity of the of the Camera-Frame, expressed in Camera-Coordinates.*/
  static constexpr unsigned int _ror = _vel+1;    /**<Idx. Angular velocity CwWC: Absolute angular velocity of the camera frame (of the solid), expressed in camera coordinates.*/
  static constexpr unsigned int _att = _ror+1;    /**<Idx. Quaternion qCW: World coordinates to Camera coordinates.*/
  StandardOutput(){
  }
  ~StandardOutput(){};

  //@{
  /** \brief Get the position vector pointing from the World-Frame to the Camera-Frame, expressed in World-Coordinates (World->%Camera, expressed in World).
   *
   *  @return the position vector WrWC (World->%Camera, expressed in World).
   */
  inline V3D& WrWC(){
    return this->template get<_pos>();
  }
  inline const V3D& WrWC() const{
    return this->template get<_pos>();
  }
  //@}

  //@{
  /** \brief Get the absolute velocity vector of the camera, expressed in camera coordinates.
   *
   *  @return the absolute velocity vector of the Camera-Frame CvC, expressed in Camera-Coordinates.
   */
  inline V3D& CvC(){
    return this->template get<_vel>();
  }
  inline const V3D& CvC() const{
    return this->template get<_vel>();
  }
  //@}

  //@{
  /** \brief Get the absolute angular velocity of the camera (angular velocity of the solid), expressed in camera coordinates.
   *
   *  @return the absolute angular velocity of the Camera-Frame CwWC, expressed in Camera-Coordinates.
   */
  inline V3D& CwWC(){
    return this->template get<_ror>();
  }
  inline const V3D& CwWC() const{
    return this->template get<_ror>();
  }
  //@}

  //@{
  /** \brief Get the quaternion qCW, expressing the  World-Frame in Camera-Coordinates (World Coordinates->Camera Coordinates).
   *
   *  @return the quaternion qCW (World Coordinates->Camera Coordinates).
   */
  inline QPD& qCW(){
    return this->template get<_att>();
  }
  inline const QPD& qCW() const{
    return this->template get<_att>();
  }
  //@}
};

template<typename STATE>
class CameraOutputCF:public LWF::CoordinateTransform<STATE,StandardOutput,true>{
 public:
  typedef LWF::CoordinateTransform<STATE,StandardOutput,true> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtOutputCovMat mtOutputCovMat;
  int camID_;
  CameraOutputCF(){
    camID_ = 0;
  };
  ~CameraOutputCF(){};
  void eval(mtOutput& output, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    // WrWC = WrWM + qWM*(MrMC)
    // CwWC = qCM*MwWM
    // CvC  = qCM*(MvM + MwWM x MrMC)
    // qCW  = qCM*qWM^T
    output.WrWC() = input.WrWM()+input.qWM().rotate(input.MrMC(camID_));
    output.CwWC() = input.qCM(camID_).rotate(V3D(input.aux().MwWMmeas_-input.gyb()));
    output.CvC()  = input.qCM(camID_).rotate(V3D(-input.MvM() + gSM(V3D(input.aux().MwWMmeas_-input.gyb()))*input.MrMC(camID_)));
    output.qCW()  = input.qCM(camID_)*input.qWM().inverted();
  }
  void jacInput(mtJacInput& J, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) = M3D::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_att>()) =
        gSM(input.qWM().rotate(input.MrMC(camID_)));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) =
        -MPD(input.qCM(camID_)*input.qWM().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vel>()) = -MPD(input.qCM(camID_)).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_gyb>()) = MPD(input.qCM(camID_)).matrix()
        * gSM(input.MrMC(camID_));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_gyb>()) = -MPD(input.qCM(camID_)).matrix();
    if(input.aux().doVECalibration_){
      J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_vep>(camID_)) =
          MPD(input.qWM()).matrix();
      J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vep>(camID_)) =
          MPD(input.qCM(camID_)).matrix()*gSM(V3D(input.aux().MwWMmeas_-input.gyb()));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          gSM(input.qCM(camID_).rotate(V3D(input.aux().MwWMmeas_-input.gyb())));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          gSM(input.qCM(camID_).rotate(V3D(-input.MvM() + gSM(V3D(input.aux().MwWMmeas_-input.gyb()))*input.MrMC())));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          M3D::Identity();
    }
  }
  void postProcess(mtOutputCovMat& cov,const mtInput& input){
    cov.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtOutput::template getId<mtOutput::_ror>()) += input.aux().wMeasCov_;
  }
};

}


#endif /* ROVIO_CAMERAOUTPUT_HPP_ */
