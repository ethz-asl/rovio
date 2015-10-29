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

#ifndef ROVIO_PIXELOUTPUT_HPP_
#define ROVIO_PIXELOUTPUT_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"

namespace rovio {

class PixelOutput: public LWF::State<LWF::VectorElement<2>>{
 public:
  static constexpr unsigned int _pix = 0;
  PixelOutput(){
  }
  virtual ~PixelOutput(){};
  cv::Point2f getPoint2f() const{
    return cv::Point2f(static_cast<float>(this->get<_pix>()(0)),static_cast<float>(this->get<_pix>()(1)));
  }
};

class PixelOutputCT:public LWF::CoordinateTransform<FeatureOutput,PixelOutput>{
 public:
  typedef LWF::CoordinateTransform<FeatureOutput,PixelOutput> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  PixelOutputCT(){
  };
  virtual ~PixelOutputCT(){};
  void evalTransform(mtOutput& output, const mtInput& input) const{
    cv::Point2f c = input.c().get_c();
    output.template get<mtOutput::_pix>() = Eigen::Vector2d(c.x,c.y);
  }
  void jacTransform(MXD& J, const mtInput& input) const{
    J.setZero();
    J.template block<2,2>(mtOutput::template getId<mtOutput::_pix>(),mtInput::template getId<mtInput::_fea>()) = input.c().get_J();
  }
};

}


#endif /* ROVIO_PIXELOUTPUT_HPP_ */
