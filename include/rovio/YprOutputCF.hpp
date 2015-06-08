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

#include "rovio/FilterStates.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

namespace rovio {

class AttitudeOutput: public LWF::State<LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _att = 0;
  AttitudeOutput(){
  }
  ~AttitudeOutput(){};
};

class YprOutput: public LWF::State<LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _ypr = 0;
  YprOutput(){
  }
  ~YprOutput(){};


};

class AttitudeToYprCF:public LWF::CoordinateTransform<AttitudeOutput,YprOutput,true>{
 public:
  typedef LWF::CoordinateTransform<AttitudeOutput,YprOutput,true> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtOutputCovMat mtOutputCovMat;
  AttitudeToYprCF(){};
  ~AttitudeToYprCF(){};
  void eval(mtOutput& output, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    output.template get<mtOutput::_ypr>() = rot::EulerAnglesYprPD(input.template get<mtInput::_att>()).vector();
  }
  void jacInput(mtJacInput& J, const mtInput& input, const mtMeas& meas, double dt = 0.0) const{
    J.setZero();

    rot::EulerAnglesYprPD ypr(input.template get<mtInput::_att>());
    const double theta = ypr.pitch();
    const double phi = ypr.roll();
    const double t2 = cos(theta);
    const double t3 = 1.0/t2;
    const double t4 = cos(phi);
    const double t5 = sin(phi);
    const double t6 = sin(theta);

    J(0,0) = 0.0;
    J(0,1) = t3*t5;
    J(0,2) = t3*t4;
    J(1,0) = 0.0;
    J(1,1) = t4;
    J(1,2) = -t5;
    J(2,0) = 1.0;
    J(2,1) = t3*t5*t6;
    J(2,2) = t3*t4*t6;
  }
};

}


#endif /* ROVIO_YPROUTPUT_HPP_ */
