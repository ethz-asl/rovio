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

#ifndef ROVIO_FEATUREOUTPUTREADABLE_HPP_
#define ROVIO_FEATUREOUTPUTREADABLE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"

namespace rovio {

class FeatureOutputReadable: public LWF::State<LWF::VectorElement<3>,LWF::ScalarElement>{
 public:
  static constexpr unsigned int _bea = 0;
  static constexpr unsigned int _dis = _bea+1;
  FeatureOutputReadable(){
    static_assert(_dis+1==E_,"Error with indices");
    this->template getName<_bea>() = "bea";
    this->template getName<_dis>() = "dis";
  }
  virtual ~FeatureOutputReadable(){};

  //@{
  /** \brief Get/Set the distance parameter
   *
   *  @return a reference to distance parameter of the feature.
   */
  inline Eigen::Vector3d& bea(){
    return this->template get<_bea>();
  }
  inline const Eigen::Vector3d& bea() const{
    return this->template get<_bea>();
  }
  //@}

  //@{
  /** \brief Get/Set the feature coordinates
   *
   *  @return a reference to the feature coordinates
   */
  inline double& dis(){
    return this->template get<_dis>();
  }
  inline const double& dis() const{
    return this->template get<_dis>();
  }
  //@}
};

class FeatureOutputReadableCT:public LWF::CoordinateTransform<FeatureOutput,FeatureOutputReadable>{
 public:
  typedef LWF::CoordinateTransform<FeatureOutput,FeatureOutputReadable> Base;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  FeatureOutputReadableCT(){};
  virtual ~FeatureOutputReadableCT(){};
  void evalTransform(mtOutput& output, const mtInput& input) const{
    output.bea() = input.c().get_nor().getVec();
    output.dis() = input.d().getDistance();
  }
  void jacTransform(MXD& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,2>(mtOutput::template getId<mtOutput::_bea>(),mtInput::template getId<mtInput::_fea>()) = input.c().get_nor().getM();
    J(mtOutput::template getId<mtOutput::_dis>(),mtInput::template getId<mtInput::_fea>()+2) = input.d().getDistanceDerivative();
  }
};

}


#endif /* ROVIO_FEATUREOUTPUTREADABLE_HPP_ */
