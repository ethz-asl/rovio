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

#ifndef ROVIO_YPROUTPUT_HPP_
#define ROVIO_YPROUTPUT_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

namespace rovio {

class AttitudeOutput: public LWF::State<LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _att = 0;
  AttitudeOutput(){
  }
  virtual ~AttitudeOutput(){};
};

class YprOutput: public LWF::State<LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _ypr = 0;
  YprOutput(){
  }
  virtual ~YprOutput(){};


};

class AttitudeToYprCT:public LWF::CoordinateTransform<AttitudeOutput,YprOutput>{
 public:
  typedef LWF::CoordinateTransform<AttitudeOutput,YprOutput> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  AttitudeToYprCT(){};
  virtual ~AttitudeToYprCT(){};
  void evalTransform(mtOutput& output, const mtInput& input) const{
    output.template get<mtOutput::_ypr>() = kindr::EulerAnglesZyxD(input.template get<mtInput::_att>()).vector();
  }
  void jacTransform(MXD& J, const mtInput& input) const{
    kindr::EulerAnglesZyxD zyx(input.template get<mtInput::_att>());
    J = zyx.getMappingFromLocalAngularVelocityToDiff()*MPD(input.template get<mtInput::_att>().inverted()).matrix();
  }
};

}


#endif /* ROVIO_YPROUTPUT_HPP_ */
