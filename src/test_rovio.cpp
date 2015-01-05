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

#include "rovio_filter.hpp"

int main(){
  static const unsigned int nMax_ = 20;
  typedef rovio::FilterState<nMax_> mtState;
  typedef rovio::PredictionMeas mtPredictionMeas;
  typedef rovio::ImgUpdateMeas<mtState> mtImgMeas;
  rovio::Filter<mtState>* mpFilter = new rovio::Filter<mtState>();
  mtState testState;
  testState.setRandom(1);
  for(unsigned int i=0;i<nMax_;i++){
    testState.template get<mtState::_aux>().addID(i+1);
  }
  for(unsigned int i=0;i<nMax_;i++){
    testState.template get<mtState::_aux>().isVisible_[i] = true;
    testState.template get<mtState::_aux>().norInCurrentFrame_[i] = testState.template get<mtState::_nor>(i);
    testState.template get<mtState::_aux>().norInCurrentFrame_[i] += Eigen::Vector3d(i*0.1,-(i*0.3)+0.03,0.1);
    testState.template get<mtState::_aux>().norInCurrentFrame_[i].normalize();
    testState.template get<mtState::_aux>().norInCurrentFrame_[i] = Eigen::Vector3d(1.0,0.3,0.5);
    testState.template get<mtState::_aux>().norInCurrentFrame_[i].normalize();
  }
  mtPredictionMeas predictionMeas_;
  predictionMeas_.setRandom(1);
  mtImgMeas imgUpdateMeas_;
  imgUpdateMeas_.setRandom(1);


//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << mpFilter->mPrediction_.jacInput(testState,predictionMeas_,0.1) << std::endl;
//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << mpFilter->mPrediction_.jacInputFD(testState,predictionMeas_,0.1,1e-6) << std::endl;
//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << mpFilter->mPrediction_.jacNoise(testState,predictionMeas_,0.1) << std::endl;
//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << mpFilter->mPrediction_.jacNoiseFD(testState,predictionMeas_,0.1,1e-6) << std::endl;
//
//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << std::get<0>(mpFilter->mUpdates_).jacInput(testState,imgUpdateMeas_,0.1) << std::endl;
//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << std::get<0>(mpFilter->mUpdates_).jacInputFD(testState,imgUpdateMeas_,0.1,1e-6) << std::endl;
//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << std::get<0>(mpFilter->mUpdates_).jacNoise(testState,imgUpdateMeas_,0.1) << std::endl;
//  std::cout << "---------------------------------------------------------------------------------" << std::endl;
//  std::cout << std::get<0>(mpFilter->mUpdates_).jacNoiseFD(testState,imgUpdateMeas_,0.1,1e-6) << std::endl;



  mpFilter->mPrediction_.testJacs(testState,predictionMeas_,1e-8,1e-6,0,0.1);
  std::get<0>(mpFilter->mUpdates_).testJacs(testState,imgUpdateMeas_,1e-8,1e-6,0,0.1);

  delete mpFilter;

  return 0;
}
