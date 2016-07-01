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

#ifndef ROVIOOUTPUT_HPP_
#define ROVIOOUTPUT_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

namespace rovio {

class StandardOutput: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,3>,LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;         /**<Idx. Position Vector WrWB: Pointing from the World-Frame to the Body-Frame, expressed in World-Coordinates.*/
  static constexpr unsigned int _vel = _pos+1;    /**<Idx. Velocity Vector  BvB: Absolute velocity of the of the Body-Frame, expressed in Body-Coordinates.*/
  static constexpr unsigned int _ror = _vel+1;    /**<Idx. Angular velocity BwWB: Absolute angular velocity of the Body frame (of the solid), expressed in Body coordinates.*/
  static constexpr unsigned int _att = _ror+1;    /**<Idx. Quaternion qBW: World coordinates to Body coordinates.*/
  StandardOutput(){
  }
  virtual ~StandardOutput(){};

  //@{
  /** \brief Get the position vector pointing from the World-Frame to the Body-Frame, expressed in World-Coordinates (World->%Body, expressed in World).
   *
   *  @return the position vector WrWB (World->Body, expressed in World).
   */
  inline V3D& WrWB(){
    return this->template get<_pos>();
  }
  inline const V3D& WrWB() const{
    return this->template get<_pos>();
  }
  //@}

  //@{
  /** \brief Get the absolute velocity vector of the camera, expressed in camera coordinates.
   *
   *  @return the absolute velocity vector of the Body-Frame BvB, expressed in Body-Coordinates.
   */
  inline V3D& BvB(){
    return this->template get<_vel>();
  }
  inline const V3D& BvB() const{
    return this->template get<_vel>();
  }
  //@}

  //@{
  /** \brief Get the absolute angular velocity of the camera (angular velocity of the solid), expressed in camera coordinates.
   *
   *  @return the absolute angular velocity of the Body-Frame BwWB, expressed in Body-Coordinates.
   */
  inline V3D& BwWB(){
    return this->template get<_ror>();
  }
  inline const V3D& BwWB() const{
    return this->template get<_ror>();
  }
  //@}

  //@{
  /** \brief Get the quaternion qBW, expressing the  World-Frame in Body-Coordinates (World Coordinates->Body Coordinates).
   *
   *  @return the quaternion qBW (World Coordinates->Body Coordinates).
   */
  inline QPD& qBW(){
    return this->template get<_att>();
  }
  inline const QPD& qBW() const{
    return this->template get<_att>();
  }
  //@}
};

template<typename STATE>
class CameraOutputCT:public LWF::CoordinateTransform<STATE,StandardOutput>{
 public:
  typedef LWF::CoordinateTransform<STATE,StandardOutput> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  int camID_;
  CameraOutputCT(){
    camID_ = 0;
  };
  virtual ~CameraOutputCT(){};
  void evalTransform(mtOutput& output, const mtInput& input) const{
    // WrWC = WrWM + qWM*(MrMC)
    // CwWC = qCM*MwWM
    // CvC  = qCM*(MvM + MwWM x MrMC)
    // qCW  = qCM*qWM^T
    output.WrWB() = input.WrWM()+input.qWM().rotate(input.MrMC(camID_));
    output.BwWB() = input.qCM(camID_).rotate(V3D(input.aux().MwWMmeas_-input.gyb()));
    output.BvB()  = input.qCM(camID_).rotate(V3D(-input.MvM() + gSM(V3D(input.aux().MwWMmeas_-input.gyb()))*input.MrMC(camID_)));
    output.qBW()  = input.qCM(camID_)*input.qWM().inverted();
  }
  void jacTransform(MXD& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) = M3D::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_att>()) =
        -gSM(input.qWM().rotate(input.MrMC(camID_)));
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
          -gSM(input.qCM(camID_).rotate(V3D(input.aux().MwWMmeas_-input.gyb())));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          -gSM(input.qCM(camID_).rotate(V3D(-input.MvM() + gSM(V3D(input.aux().MwWMmeas_-input.gyb()))*input.MrMC())));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          M3D::Identity();
    }
  }
  void postProcess(MXD& cov,const mtInput& input){
    cov.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtOutput::template getId<mtOutput::_ror>()) += input.aux().wMeasCov_;
  }
};

template<typename STATE>
class ImuOutputCT:public LWF::CoordinateTransform<STATE,StandardOutput>{
 public:
  typedef LWF::CoordinateTransform<STATE,StandardOutput> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  ImuOutputCT(){};
  virtual ~ImuOutputCT(){};
  void evalTransform(mtOutput& output, const mtInput& input) const{
    // WrWM = WrWM
    // MwWM = MwWM
    // MvM  = MvM
    // qMW  = qWM^T
    output.WrWB() = input.WrWM();
    output.BwWB() = input.aux().MwWMmeas_-input.gyb();
    output.BvB()  = -input.MvM(); // minus is required!
    output.qBW()  = input.qWM().inverted();
  }
  void jacTransform(MXD& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) = M3D::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) = -MPD(input.qWM().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vel>()) = -M3D::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_gyb>()) = -M3D::Identity();
  }
  void postProcess(MXD& cov,const mtInput& input){
    cov.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtOutput::template getId<mtOutput::_ror>()) += input.aux().wMeasCov_;
  }
};

}


#endif /* ROVIOOUTPUT_HPP_ */
