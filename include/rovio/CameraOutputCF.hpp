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
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _ror = _vel+1;
  static constexpr unsigned int _att = _ror+1;
  StandardOutput(){
  }
  ~StandardOutput(){};
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
    // IrIV = IrIM + qIM*(MrMV)
    // VvV = qVM*(MvM + MwIM x MrMV)
    // qVI = qVM*qIM^T
    // VwIV = qVM*MwIM
    output.template get<mtOutput::_pos>() = input.template get<mtInput::_pos>()+input.template get<mtInput::_att>().rotate(input.get_MrMV(camID_));
    output.template get<mtOutput::_ror>() = input.get_qVM(camID_).rotate(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()));
    output.template get<mtOutput::_vel>() =
        input.get_qVM(camID_).rotate(V3D(-input.template get<mtInput::_vel>() + gSM(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()))*input.get_MrMV(camID_)));
    output.template get<mtOutput::_att>() = input.get_qVM(camID_)*input.template get<mtInput::_att>().inverted();
  }
  void jacInput(mtJacInput& J, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) = M3D::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_att>()) =
        gSM(input.template get<mtInput::_att>().rotate(input.get_MrMV(camID_)));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) =
        -MPD(input.get_qVM(camID_)*input.template get<mtInput::_att>().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vel>()) = -MPD(input.get_qVM(camID_)).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_gyb>()) = MPD(input.get_qVM(camID_)).matrix()
        * gSM(input.get_MrMV(camID_));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_gyb>()) = -MPD(input.get_qVM(camID_)).matrix();
    if(input.template get<mtInput::_aux>().doVECalibration_){
      J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_vep>(camID_)) =
          MPD(input.template get<mtInput::_att>()).matrix();
      J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vep>(camID_)) =
          MPD(input.get_qVM(camID_)).matrix()*gSM(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          gSM(input.get_qVM(camID_).rotate(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>())));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          gSM(input.get_qVM(camID_).rotate(V3D(-input.template get<mtInput::_vel>() + gSM(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()))*input.get_MrMV())));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_vea>(camID_)) =
          M3D::Identity();
    }
  }
  void postProcess(mtOutputCovMat& cov,const mtInput& input){
    cov.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtOutput::template getId<mtOutput::_ror>()) += input.template get<mtInput::_aux>().wMeasCov_;
  }
};

}


#endif /* ROVIO_CAMERAOUTPUT_HPP_ */
