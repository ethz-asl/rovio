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

#ifndef ROVIO_IMGUPDATE_HPP_
#define ROVIO_IMGUPDATE_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/Camera.hpp"
#include "rovio/PixelOutputCF.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

template<typename STATE>
class ImgInnovation: public LWF::State<LWF::VectorElement<2>>{
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgInnovation(){
    static_assert(_nor+1==E_,"Error with indices");
    this->template getName<_nor>() = "nor";
  };
  ~ImgInnovation(){};
};
template<typename STATE>
class ImgUpdateMeasAuxiliary: public LWF::AuxiliaryBase<ImgUpdateMeasAuxiliary<STATE>>{
 public:
  ImgUpdateMeasAuxiliary(){
    imgTime_ = 0.0;
  };
  ~ImgUpdateMeasAuxiliary(){};
  ImagePyramid<STATE::nLevels_> pyr_;
  double imgTime_;
};
template<typename STATE>
class ImgUpdateMeas: public LWF::State<ImgUpdateMeasAuxiliary<STATE>>{
 public:
  typedef LWF::State<ImgUpdateMeasAuxiliary<STATE>> Base;
  using Base::E_;
  static constexpr unsigned int _aux = 0;
  ImgUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  ~ImgUpdateMeas(){};
};
template<typename STATE>
class ImgUpdateNoise: public LWF::State<LWF::VectorElement<2>>{
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgUpdateNoise(){
    static_assert(_nor+1==E_,"Error with indices");
    this->template getName<_nor>() = "nor";
  };
  ~ImgUpdateNoise(){};
};
template<typename STATE>
class ImgOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_nor>(),2>>{
};

template<typename FILTERSTATE>
class ImgUpdate: public LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
                                    ImgOutlierDetection<typename FILTERSTATE::mtState>,false>{
 public:
  typedef LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
                      ImgOutlierDetection<typename FILTERSTATE::mtState>,false> Base;
  using typename Base::eval;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::boolRegister_;
  using Base::updnoiP_;
  using Base::useSpecialLinearizationPoint_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  M3D initCovFeature_;
  double initDepth_;
  Camera* mpCamera_;
  PixelOutputCF<typename FILTERSTATE::mtState> pixelOutputCF_;
  PixelOutput pixelOutput_;
  typename PixelOutputCF<typename FILTERSTATE::mtState>::mtOutputCovMat pixelOutputCov_;
  int startLevel_;
  int endLevel_;
  double startDetectionTh_;
  int nDetectionBuckets_;
  int fastDetectionThreshold_;
  double scoreDetectionExponent_;
  double penaltyDistance_;
  double zeroDistancePenalty_;
  double trackingLocalRange_,trackingLocalVisibilityRange_;
  double trackingUpperBound_,trackingLowerBound_;
  double minTrackedAndFreeFeatures_;
  double minRelativeSTScore_;
  double minAbsoluteSTScore_;
  double matchingPixelThreshold_;
  bool doPatchWarping_;
  bool useDirectMethod_;
  bool doFrameVisualisation_;
  bool verbose_;
  ImgUpdate(){
    mpCamera_ = nullptr;
    initCovFeature_.setIdentity();
    initDepth_ = 0;
    startLevel_ = 3;
    endLevel_ = 1;
    startDetectionTh_ = 0.9;
    nDetectionBuckets_ = 100;
    fastDetectionThreshold_ = 10;
    scoreDetectionExponent_ = 0.5;
    penaltyDistance_ = 20;
    zeroDistancePenalty_ = nDetectionBuckets_*1.0;
    doPatchWarping_ = true;
    useDirectMethod_ = true;
    doFrameVisualisation_ = true;
    verbose_ = false; // TODO: register
    trackingLocalRange_ = 20;
    trackingLocalVisibilityRange_ = 200;
    trackingUpperBound_ = 0.9;
    trackingLowerBound_ = 0.1;
    minTrackedAndFreeFeatures_ = 0.5;
    minRelativeSTScore_ = 0.2;
    minAbsoluteSTScore_ = 0.2;
    matchingPixelThreshold_ = 4.0;
    doubleRegister_.registerDiagonalMatrix("initCovFeature",initCovFeature_);
    doubleRegister_.registerScalar("initDepth",initDepth_);
    doubleRegister_.registerScalar("startDetectionTh",startDetectionTh_);
    doubleRegister_.registerScalar("scoreDetectionExponent",scoreDetectionExponent_);
    doubleRegister_.registerScalar("penaltyDistance",penaltyDistance_);
    doubleRegister_.registerScalar("zeroDistancePenalty",zeroDistancePenalty_);
    doubleRegister_.registerScalar("trackingLocalRange",trackingLocalRange_);
    doubleRegister_.registerScalar("trackingLocalVisibilityRange",trackingLocalVisibilityRange_);
    doubleRegister_.registerScalar("trackingUpperBound",trackingUpperBound_);
    doubleRegister_.registerScalar("trackingLowerBound",trackingLowerBound_);
    doubleRegister_.registerScalar("minTrackedAndFreeFeatures",minTrackedAndFreeFeatures_);
    doubleRegister_.registerScalar("minRelativeSTScore",minRelativeSTScore_);
    doubleRegister_.registerScalar("minAbsoluteSTScore",minAbsoluteSTScore_);
    doubleRegister_.registerScalar("matchingPixelThreshold",matchingPixelThreshold_);
    intRegister_.registerScalar("fastDetectionThreshold",fastDetectionThreshold_);
    intRegister_.registerScalar("startLevel",startLevel_);
    intRegister_.registerScalar("endLevel",endLevel_);
    intRegister_.registerScalar("nDetectionBuckets",nDetectionBuckets_);
    boolRegister_.registerScalar("doPatchWarping",doPatchWarping_);
    boolRegister_.registerScalar("useDirectMethod",useDirectMethod_);
    doubleRegister_.removeScalarByVar(updnoiP_(0,0));
    doubleRegister_.removeScalarByVar(updnoiP_(1,1));
    doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(0,0));
    doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(1,1));
  };
  ~ImgUpdate(){};
  void refreshProperties(){
    useSpecialLinearizationPoint_ = useDirectMethod_;
  };
  void setCamera(Camera* mpCamera){
    mpCamera_ = mpCamera;
    pixelOutputCF_.setCamera(mpCamera);
  }
  void eval(mtInnovation& y, const mtState& state, const mtMeas& meas, const mtNoise noise, double dt = 0.0) const{
    const int& ID = state.template get<mtState::_aux>().activeFeature_;
    if(useDirectMethod_){
      y.template get<mtInnovation::_nor>() = state.template get<mtState::_aux>().b_red_[ID]+noise.template get<mtNoise::_nor>();
    } else {
      state.template get<mtState::_nor>(ID).boxMinus(state.template get<mtState::_aux>().bearingMeas_[ID],y.template get<mtInnovation::_nor>()); // 0 = m - m_meas + n
      y.template get<mtInnovation::_nor>() += noise.template get<mtNoise::_nor>();
    }
  }
  void jacInput(mtJacInput& F, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    const int& ID = state.template get<mtState::_aux>().activeFeature_;
    F.setZero();
    cv::Point2f c_temp;
    Eigen::Matrix2d c_J;
    if(useDirectMethod_){
      mpCamera_->bearingToPixel(state.template get<mtState::_nor>(ID),c_temp,c_J);
      F.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(),mtState::template getId<mtState::_nor>(ID)) = -state.template get<mtState::_aux>().A_red_[ID]*c_J;
    } else {
      F.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(),mtState::template getId<mtState::_nor>(ID)) =
            state.template get<mtState::_aux>().bearingMeas_[ID].getN().transpose()
            *-LWF::NormalVectorElement::getRotationFromTwoNormalsJac(state.template get<mtState::_nor>(ID),state.template get<mtState::_aux>().bearingMeas_[ID])
            *state.template get<mtState::_nor>(ID).getM();
    }
  }
  void jacNoise(mtJacNoise& G, const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    G.setZero();
    G.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(),mtNoise::template getId<mtNoise::_nor>()) = Eigen::Matrix2d::Identity();
  }
  void commonPreProcess(mtFilterState& filterState, const mtMeas& meas){
    assert(filterState.t_ == meas.template get<mtMeas::_aux>().imgTime_);
    if(doFrameVisualisation_){
      cvtColor(meas.template get<mtMeas::_aux>().pyr_.imgs_[0], filterState.img_, CV_GRAY2RGB);
    }
    filterState.imgTime_ = filterState.t_;
    filterState.imageCounter_++;
    filterState.patchDrawing_ = cv::Mat::zeros(mtState::nMax_*pow(2,mtState::nLevels_-1),mtState::nMax_*pow(2,mtState::nLevels_-1),CV_8UC1); // TODO
    filterState.state_.template get<mtState::_aux>().activeFeature_ = 0;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      filterState.state_.template get<mtState::_aux>().useInUpdate_[i] = false;
    }

    // TODO sort feature by covariance and use more accurate ones first
  }
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished){
    if(isFinished){ // gets called if this is the first call
      commonPreProcess(filterState,meas);
      isFinished = false;
    }
    typename mtFilterState::mtState& state = filterState.state_;
    typename mtFilterState::mtFilterCovMat& cov = filterState.cov_;
    int& ID = filterState.state_.template get<mtState::_aux>().activeFeature_;

    MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_>* mpFeature;
    Eigen::Vector2d vec2;
    while(ID < mtState::nMax_ && state.template get<mtState::_aux>().useInUpdate_[ID] == false){
      if(filterState.mlps_.isValid_[ID]){
        mpFeature = &filterState.mlps_.features_[ID];
        // Data handling stuff
        mpFeature->set_nor(state.template get<mtState::_nor>(ID));
        mpFeature->increaseStatistics(filterState.t_);

        // Check if prediction in frame
        mpFeature->status_.inFrame_ = isMultilevelPatchInFrame(*mpFeature,meas.template get<mtMeas::_aux>().pyr_,startLevel_,false,doPatchWarping_);
        if(mpFeature->status_.inFrame_){
          pixelOutputCF_.setIndex(ID);
          pixelOutputCF_.transformCovMat(state,cov,pixelOutputCov_);
          mpFeature->setSigmaFromCov(pixelOutputCov_);
          mpFeature->log_prediction_ = static_cast<FeatureCoordinates>(*mpFeature);
          mpFeature->set_bearingCorners(state.template get<mtState::_aux>().bearingCorners_[ID]);
          const PixelCorners& pixelCorners = mpFeature->get_pixelCorners();
          mpFeature->log_predictionC0_.set_c(mpFeature->get_c() - 4*pixelCorners[0] - 4*pixelCorners[1]);
          mpFeature->log_predictionC1_.set_c(mpFeature->get_c() + 4*pixelCorners[0] - 4*pixelCorners[1]);
          mpFeature->log_predictionC2_.set_c(mpFeature->get_c() - 4*pixelCorners[0] + 4*pixelCorners[1]);
          mpFeature->log_predictionC3_.set_c(mpFeature->get_c() + 4*pixelCorners[0] + 4*pixelCorners[1]);

          // Search patch // TODO: do adaptive
          if(!useDirectMethod_ || pixelOutputCov_.operatorNorm() > matchingPixelThreshold_){
            align2DComposed(*mpFeature,meas.template get<mtMeas::_aux>().pyr_,startLevel_,endLevel_,startLevel_-endLevel_,doPatchWarping_);
          }
          if(mpFeature->status_.matchingStatus_ == FOUND) mpFeature->log_meas_.set_c(mpFeature->get_c());

          // Add as measurement
          if(useDirectMethod_){
            vec2.setZero();
            if(mpFeature->status_.matchingStatus_ == FOUND){
              mpFeature->get_nor().boxMinus(state.template get<mtState::_nor>(ID),vec2);
              if((vec2.transpose()*cov.template block<2,2>(mtState::template getId<mtState::_nor>(ID),mtState::template getId<mtState::_nor>(ID)).inverse()*vec2)(0,0) > 5.886){ // TODO: param
                mpFeature->set_nor(state.template get<mtState::_nor>(ID));
                vec2.setZero();
              }
            }
            filterState.difVecLin_.template block<2,1>(mtState::template getId<mtState::_nor>(ID),0) = vec2;
            if(getLinearAlignEquationsReduced(*mpFeature,meas.template get<mtMeas::_aux>().pyr_,endLevel_,startLevel_,doPatchWarping_,
                                              state.template get<mtState::_aux>().A_red_[ID],state.template get<mtState::_aux>().b_red_[ID])){
              state.template get<mtState::_aux>().useInUpdate_[ID] = true;
            }
          } else {
            if(mpFeature->status_.matchingStatus_ == FOUND){
              state.template get<mtState::_aux>().useInUpdate_[ID] = true;
              state.template get<mtState::_aux>().bearingMeas_[ID] = mpFeature->get_nor();
            }
          }
        }
      }
      if(state.template get<mtState::_aux>().useInUpdate_[ID] == false){
        ID++;
      }
    }
    if(ID >= mtState::nMax_){
      isFinished = true;
    }
  };
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    if(isFinished){
      commonPostProcess(filterState,meas);
    } else {
      int& ID = filterState.state_.template get<mtState::_aux>().activeFeature_;
      if(!outlierDetection.isOutlier(0)){
        filterState.mlps_.features_[ID].status_.trackingStatus_ = TRACKED;
      } else {
        filterState.mlps_.features_[ID].status_.trackingStatus_ = FAILED;
      }
      ID++;
    }
  };
  void commonPostProcess(mtFilterState& filterState, const mtMeas& meas){
    // Temps
    float averageScore;
    int countTracked;
    int requiredFreeFeature;
    double removalFactor;
    int featureIndex;
    typename mtFilterState::mtState& state = filterState.state_;
    typename mtFilterState::mtFilterCovMat& cov = filterState.cov_;
    MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_>* mpFeature;
    MultilevelPatchFeature<mtState::nLevels_,mtState::patchSize_> testFeature;

    countTracked = 0;
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.mlps_.isValid_[i]){
        mpFeature = &filterState.mlps_.features_[i];
        if(mpFeature->status_.inFrame_){
          // Logging
          mpFeature->set_nor(state.template get<mtState::_nor>(i));
          pixelOutputCF_.setIndex(i);
          pixelOutputCF_.transformCovMat(state,cov,pixelOutputCov_);
          mpFeature->setSigmaFromCov(pixelOutputCov_);
          mpFeature->log_current_ = static_cast<FeatureCoordinates>(*mpFeature);

          // Count Tracked
          if(mpFeature->status_.trackingStatus_ == TRACKED){
            countTracked++;
          }

          // Extract feature patches
          if(mpFeature->status_.trackingStatus_ == TRACKED){
            if(isMultilevelPatchInFrame(*mpFeature,meas.template get<mtMeas::_aux>().pyr_,startLevel_,true,false)){
              testFeature.set_c(mpFeature->get_c());
              extractMultilevelPatchFromImage(testFeature,meas.template get<mtMeas::_aux>().pyr_,startLevel_,true,false);
              testFeature.computeMultilevelShiTomasiScore(endLevel_,startLevel_);
              if(testFeature.s_ >= static_cast<float>(minAbsoluteSTScore_) || testFeature.s_ >= static_cast<float>(minRelativeSTScore_)*mpFeature->s_){
                extractMultilevelPatchFromImage(*mpFeature,meas.template get<mtMeas::_aux>().pyr_,startLevel_,true,false);
                mpFeature->computeMultilevelShiTomasiScore(endLevel_,startLevel_);
                state.template get<mtState::_aux>().bearingCorners_[i] = mpFeature->get_bearingCorners();
              }
            }
          }

          // Drawing
          if(mpFeature->status_.inFrame_ && doFrameVisualisation_){
            drawEllipse(filterState.img_,mpFeature->log_prediction_,cv::Scalar(0,175,175));
            drawLine(filterState.img_,mpFeature->log_predictionC0_,mpFeature->log_predictionC1_,cv::Scalar(0,175,175),1);
            drawLine(filterState.img_,mpFeature->log_predictionC0_,mpFeature->log_predictionC2_,cv::Scalar(0,175,175),1);
            drawLine(filterState.img_,mpFeature->log_predictionC3_,mpFeature->log_predictionC1_,cv::Scalar(0,175,175),1);
            drawLine(filterState.img_,mpFeature->log_predictionC3_,mpFeature->log_predictionC2_,cv::Scalar(0,175,175),1);
            if(mpFeature->status_.trackingStatus_ == TRACKED){
              drawLine(filterState.img_,mpFeature->log_current_,mpFeature->log_prediction_,cv::Scalar(0,255,0));
              drawEllipse(filterState.img_,mpFeature->log_current_,cv::Scalar(0, 255, 0));
              drawText(filterState.img_,mpFeature->log_current_,std::to_string(mpFeature->totCount_),cv::Scalar(0,255,0));
            } else if(mpFeature->status_.trackingStatus_ == FAILED){
              drawEllipse(filterState.img_,mpFeature->log_current_,cv::Scalar(0, 0, 255));
              drawText(filterState.img_,mpFeature->log_current_,std::to_string(mpFeature->countTrackingStatistics(FAILED,trackingLocalRange_)),cv::Scalar(0,0,255));
            } else {
              drawEllipse(filterState.img_,mpFeature->log_current_,cv::Scalar(0,255,255));
              drawText(filterState.img_,mpFeature->log_current_,std::to_string(mpFeature->countTrackingStatistics(NOTTRACKED,trackingLocalVisibilityRange_)),cv::Scalar(0,255,255));
            }
          }
        }
      }
    }

    // Remove bad feature
    averageScore = filterState.mlps_.getAverageScore(); // TODO
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.mlps_.isValid_[i]){
        mpFeature = &filterState.mlps_.features_[i];
        if(!mpFeature->isGoodFeature(trackingLocalRange_,trackingLocalVisibilityRange_,trackingUpperBound_,trackingLowerBound_)){
          //          || fManager.features_[ind].s_ < static_cast<float>(minAbsoluteSTScore_) + static_cast<float>(minRelativeSTScore_)*averageScore){ //TODO: debug and fix
          filterState.mlps_.isValid_[i] = false;
          filterState.removeFeature(i);
        }
      }
    }

    // Check if enough free features
    requiredFreeFeature = mtState::nMax_*minTrackedAndFreeFeatures_-countTracked;
    removalFactor = 1.1; // TODO: param
    featureIndex = 0;
    while((int)(mtState::nMax_) - (int)(filterState.mlps_.getValidCount()) < requiredFreeFeature){
      if(filterState.mlps_.isValid_[featureIndex]){
        mpFeature = &filterState.mlps_.features_[featureIndex];
        if(mpFeature->status_.trackingStatus_ != TRACKED &&
          !mpFeature->isGoodFeature(trackingLocalRange_,trackingLocalVisibilityRange_,trackingUpperBound_*removalFactor,trackingLowerBound_*removalFactor)){ // TODO: improve
          filterState.mlps_.isValid_[featureIndex] = false;
          filterState.removeFeature(featureIndex);
        }
      }
      featureIndex++;
      if(featureIndex == mtState::nMax_){
        featureIndex = 0;
        removalFactor = removalFactor*1.1; // TODO: param
      }
    }

    // Get new features
    averageScore = filterState.mlps_.getAverageScore(); // TODO
    if(filterState.mlps_.getValidCount() < startDetectionTh_*mtState::nMax_){
      std::list<cv::Point2f> candidates;
      if(verbose_) std::cout << "Adding keypoints" << std::endl;
      const double t1 = (double) cv::getTickCount();
      for(int l=endLevel_;l<=startLevel_;l++){
        detectFastCorners(meas.template get<mtMeas::_aux>().pyr_,candidates,l,fastDetectionThreshold_);
      }
      const double t2 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Detected " << candidates.size() << " on levels " << endLevel_ << "-" << startLevel_ << " (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      pruneCandidates(filterState.mlps_,candidates);
      const double t3 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Selected " << candidates.size() << " candidates (" << (t3-t2)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      std::unordered_set<unsigned int> newSet = addBestCandidates(filterState.mlps_,candidates,meas.template get<mtMeas::_aux>().pyr_,filterState.t_,
                                                                  endLevel_,startLevel_,mtState::nMax_-filterState.mlps_.getValidCount(),nDetectionBuckets_, scoreDetectionExponent_,
                                                                  penaltyDistance_, zeroDistancePenalty_,false,0.0);
      const double t4 = (double) cv::getTickCount();
      if(verbose_) std::cout << "== Got " << filterState.mlps_.getValidCount() << " after adding " << newSet.size() << " features (" << (t4-t3)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
      for(auto it = newSet.begin();it != newSet.end();++it){
        filterState.mlps_.features_[*it].setCamera(mpCamera_);
        filterState.mlps_.features_[*it].status_.inFrame_ = true;
        filterState.mlps_.features_[*it].status_.matchingStatus_ = FOUND;
        filterState.mlps_.features_[*it].status_.trackingStatus_ = TRACKED;
        filterState.initializeFeatureState(*it,filterState.mlps_.features_[*it].get_nor().getVec(),initDepth_,initCovFeature_);
        state.template get<mtState::_aux>().bearingCorners_[*it] = filterState.mlps_.features_[*it].get_bearingCorners();
      }
    }
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.mlps_.isValid_[i] && filterState.mlps_.features_[i].status_.inFrame_){
        filterState.mlps_.features_[i].log_previous_ = static_cast<FeatureCoordinates>(filterState.mlps_.features_[i]);
      }
    }
    if (doFrameVisualisation_){
      drawVirtualHorizon(filterState);
    }

  }
  void drawVirtualHorizon(mtFilterState& filterState){
    typename mtFilterState::mtState& state = filterState.state_;
    cv::rectangle(filterState.img_,cv::Point2f(0,0),cv::Point2f(82,92),cv::Scalar(50,50,50),-1,8,0);
    cv::rectangle(filterState.img_,cv::Point2f(0,0),cv::Point2f(80,90),cv::Scalar(100,100,100),-1,8,0);
    cv::putText(filterState.img_,std::to_string(filterState.imageCounter_),cv::Point2f(5,85),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,0));
    cv::Point2f rollCenter = cv::Point2f(40,40);
    cv::Scalar rollColor1(50,50,50);
    cv::Scalar rollColor2(200,200,200);
    cv::Scalar rollColor3(120,120,120);
    cv::circle(filterState.img_,rollCenter,32,rollColor1,-1,8,0);
    cv::circle(filterState.img_,rollCenter,30,rollColor2,-1,8,0);
    Eigen::Vector3d Vg = (state.template get<mtState::_vea>()*state.template get<mtState::_att>().inverted()).rotate(Eigen::Vector3d(0,0,-1));
    double roll = atan2(Vg(1),Vg(0))-0.5*M_PI;
    double pitch = acos(Vg.dot(Eigen::Vector3d(0,0,1)))-0.5*M_PI;
    double pixelFor10Pitch = 5.0;
    double pitchOffsetAngle = -asin(pitch/M_PI*180.0/10.0*pixelFor10Pitch/30.0);
    cv::Point2f rollVector1 = 30*cv::Point2f(cos(roll),sin(roll));
    cv::Point2f rollVector2 = cv::Point2f(25,0);
    cv::Point2f rollVector3 = cv::Point2f(10,0);
    std::vector<cv::Point> pts;
    cv::ellipse2Poly(rollCenter,cv::Size(30,30),0,(roll-pitchOffsetAngle)/M_PI*180,(roll+pitchOffsetAngle)/M_PI*180+180,1,pts);
    cv::Point *points;
    points = &pts[0];
    int nbtab = pts.size();
    cv::fillPoly(filterState.img_,(const cv::Point**)&points,&nbtab,1,rollColor3);
    cv::line(filterState.img_,rollCenter+rollVector2,rollCenter+rollVector3,rollColor1, 2);
    cv::line(filterState.img_,rollCenter-rollVector2,rollCenter-rollVector3,rollColor1, 2);
    cv::ellipse(filterState.img_,rollCenter,cv::Size(10,10),0,0,180,rollColor1,2,8,0);
    cv::circle(filterState.img_,rollCenter,2,rollColor1,-1,8,0);
  }
};

}


#endif /* ROVIO_IMGUPDATE_HPP_ */
