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

#ifndef IMGUPDATE_HPP_
#define IMGUPDATE_HPP_

#include "kindr/rotations/RotationEigen.hpp"
#include <Eigen/Dense>
#include "Update.hpp"
#include "State.hpp"
#include "FilterState.hpp"
#include <cv_bridge/cv_bridge.h>
#include "Camera.hpp"
#include "PixelOutputCF.hpp"

namespace rot = kindr::rotations::eigen_impl;

namespace rovio {

using namespace LWF;

template<typename STATE>
class ImgInnovation: public State<ArrayElement<VectorElement<2>,STATE::nMax_>>{
 public:
  typedef State<ArrayElement<VectorElement<2>,STATE::nMax_>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgInnovation(){
    static_assert(_nor+1==E_,"Error with indices");
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
class ImgUpdateMeas: public State<ImgUpdateMeasAuxiliary<STATE>>{
 public:
  typedef State<ImgUpdateMeasAuxiliary<STATE>> Base;
  using Base::E_;
  static constexpr unsigned int _aux = 0;
  ImgUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  ~ImgUpdateMeas(){};
};
template<typename STATE>
class ImgUpdateNoise: public State<ArrayElement<VectorElement<2>,STATE::nMax_>>{
 public:
  typedef State<ArrayElement<VectorElement<2>,STATE::nMax_>> Base;
  using Base::E_;
  static constexpr unsigned int _nor = 0;
  ImgUpdateNoise(){
    static_assert(_nor+1==E_,"Error with indices");
    this->template getName<_nor>() = "nor";
  };
  ~ImgUpdateNoise(){};
};
template<typename STATE>
class ImgOutlierDetection: public OutlierDetection<ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_nor>(),2,STATE::nMax_>>{
};

template<typename STATE>
class ImgUpdate: public Update<ImgInnovation<STATE>,STATE,ImgUpdateMeas<STATE>,ImgUpdateNoise<STATE>,
    ImgOutlierDetection<STATE>,DummyPrediction,false>{
 public:
  typedef Update<ImgInnovation<STATE>,STATE,ImgUpdateMeas<STATE>,ImgUpdateNoise<STATE>,
      ImgOutlierDetection<STATE>,DummyPrediction,false> Base;
  using typename Base::eval;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::boolRegister_;
  using Base::updnoiP_;
  using Base::useSpecialLinearizationPoint_;
  using Base::difVecLin_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtCovMat mtCovMat;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtJacInput mtJacInput;
  typedef typename Base::mtJacNoise mtJacNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  M3D initCovFeature_;
  double initDepth_;
  Camera* mpCamera_;
  PixelOutputCF<STATE> pixelOutputCF_;
  PixelOutput pixelOutput_;
  int startLevel_;
  int endLevel_;
  typename PixelOutput::mtCovMat pixelOutputCov_;
  double startDetectionTh_;
  int nDetectionBuckets_;
  double scoreDetectionExponent_;
  double penaltyDistance_;
  double zeroDistancePenalty_;
  double trackingLocalRange_,trackingLocalVisibilityRange_;
  double trackingUpperBound_,trackingLowerBound_;
  double minTrackedAndFreeFeatures_;
  double minRelativeSTScore_;
  double minAbsoluteSTScore_;
  bool doPatchWarping_;
  bool useDirectMethod_;
  ImgUpdate(){
    mpCamera_ = nullptr;
    initCovFeature_.setIdentity();
    initDepth_ = 0;
    startLevel_ = 3;
    endLevel_ = 1;
    startDetectionTh_ = 0.9;
    nDetectionBuckets_ = 100;
    scoreDetectionExponent_ = 0.5;
    penaltyDistance_ = 20;
    zeroDistancePenalty_ = nDetectionBuckets_*1.0;
    doPatchWarping_ = true;
    useDirectMethod_ = true;
    trackingLocalRange_ = 20;
    trackingLocalVisibilityRange_ = 200;
    trackingUpperBound_ = 0.9;
    trackingLowerBound_ = 0.1;
    minTrackedAndFreeFeatures_ = 0.5;
    minRelativeSTScore_ = 0.2;
    minAbsoluteSTScore_ = 0.2;
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
    intRegister_.registerScalar("startLevel",startLevel_);
    intRegister_.registerScalar("endLevel",endLevel_);
    intRegister_.registerScalar("nDetectionBuckets",nDetectionBuckets_);
    boolRegister_.registerScalar("doPatchWarping",doPatchWarping_);
    boolRegister_.registerScalar("useDirectMethod",useDirectMethod_);
    int ind;
    for(int i=0;i<STATE::nMax_;i++){
      ind = mtNoise::template getId<mtNoise::_nor>(i);
      doubleRegister_.removeScalarByVar(updnoiP_(ind,ind));
      doubleRegister_.removeScalarByVar(updnoiP_(ind+1,ind+1));
      doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(ind,ind));
      doubleRegister_.registerScalar("UpdateNoise.nor",updnoiP_(ind+1,ind+1));
    }
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
    FeatureManager<STATE::nLevels_,STATE::patchSize_,mtState::nMax_>& fManager = const_cast<FeatureManager<STATE::nLevels_,STATE::patchSize_,mtState::nMax_>&>(state.template get<mtState::_aux>().fManager_);
//    /* Bearing vector error
//     * 0 = m - m_meas + n
//     */
    NormalVectorElement m_meas;
    Eigen::Matrix2d A_red;
    Eigen::Vector2d b_red;
    cv::Point2f c_temp;
    Eigen::Matrix2d c_J;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      state.template get<mtState::_nor>(ind).boxPlus(state.difVecLin_.template block<2,1>(mtState::template getId<mtState::_nor>(ind),0),m_meas);
      mpCamera_->bearingToPixel(m_meas,c_temp,c_J);
      if(useDirectMethod_ && fManager.features_[ind].currentStatistics_.inFrame_){
        if(fManager.features_[ind].getLinearAlignEquationsReduced(meas.template get<mtMeas::_aux>().pyr_,c_temp,endLevel_,startLevel_,doPatchWarping_,A_red,b_red)){
          y.template get<mtInnovation::_nor>(ind) = b_red+noise.template get<mtNoise::_nor>(ind);
        } else {
          y.template get<mtInnovation::_nor>(ind) = noise.template get<mtNoise::_nor>(ind);
          fManager.features_[ind].currentStatistics_.inFrame_ = false; //TODO: should not be changed here
        }
      } else if(!useDirectMethod_ && fManager.features_[ind].currentStatistics_.status_ == TrackingStatistics::FOUND){
        state.template get<mtState::_nor>(ind).boxMinus(m_meas,y.template get<mtInnovation::_nor>(ind));
        y.template get<mtInnovation::_nor>(ind) += noise.template get<mtNoise::_nor>(ind);
      } else {
        y.template get<mtInnovation::_nor>(ind) = noise.template get<mtNoise::_nor>(ind);
      }
    }
    for(auto it_f = fManager.invalidSet_.begin();it_f != fManager.invalidSet_.end(); ++it_f){
      const int ind = *it_f;
      y.template get<mtInnovation::_nor>(ind) = noise.template get<mtNoise::_nor>(ind);
    }
  }
  mtJacInput jacInput(const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    FeatureManager<STATE::nLevels_,STATE::patchSize_,mtState::nMax_>& fManager = const_cast<FeatureManager<STATE::nLevels_,STATE::patchSize_,mtState::nMax_>&>(state.template get<mtState::_aux>().fManager_);
    mtJacInput J;
    J.setZero();
    NormalVectorElement m_meas;
    Eigen::Matrix2d A_red;
    Eigen::Vector2d b_red;
    cv::Point2f c_temp;
    Eigen::Matrix2d c_J;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      state.template get<mtState::_nor>(ind).boxPlus(state.difVecLin_.template block<2,1>(mtState::template getId<mtState::_nor>(ind),0),m_meas);
      mpCamera_->bearingToPixel(m_meas,c_temp,c_J);
      if(useDirectMethod_ && fManager.features_[ind].currentStatistics_.inFrame_){
        if(fManager.features_[ind].getLinearAlignEquationsReduced(meas.template get<mtMeas::_aux>().pyr_,c_temp,endLevel_,startLevel_,doPatchWarping_,A_red,b_red)){
          J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(ind),mtState::template getId<mtState::_nor>(ind)) = -A_red*c_J;
        } else {
          fManager.features_[ind].currentStatistics_.inFrame_ = false; //TODO: should not be changed here
        }
      } else if(!useDirectMethod_ && fManager.features_[ind].currentStatistics_.status_ == TrackingStatistics::FOUND){
        J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(ind),mtState::template getId<mtState::_nor>(ind)) =
              m_meas.getN().transpose()
              *-LWF::NormalVectorElement::getRotationFromTwoNormalsJac(state.template get<mtState::_nor>(ind),m_meas)
              *state.template get<mtState::_nor>(ind).getM();
      }
    }
    return J;
  }
  mtJacNoise jacNoise(const mtState& state, const mtMeas& meas, double dt = 0.0) const{
    mtJacNoise J;
    J.setZero();
    for(unsigned int i=0;i<STATE::nMax_;i++){
      J.template block<2,2>(mtInnovation::template getId<mtInnovation::_nor>(i),mtNoise::template getId<mtNoise::_nor>(i)) = Eigen::Matrix2d::Identity();
    }
    return J;
  }
  void preProcess(mtState& state, mtCovMat& cov, const mtMeas& meas){
    double lastImageTime = 0.0;
    if(!state.template get<mtState::_aux>().img_.empty()){
      lastImageTime = state.template get<mtState::_aux>().imgTime_;
    }
    cvtColor(meas.template get<mtMeas::_aux>().pyr_.imgs_[0], state.template get<mtState::_aux>().img_, CV_GRAY2RGB);
    state.template get<mtState::_aux>().imgTime_ = meas.template get<mtMeas::_aux>().imgTime_;
    FeatureManager<STATE::nLevels_,STATE::patchSize_,mtState::nMax_>& fManager = state.template get<mtState::_aux>().fManager_;
    state.template get<mtState::_aux>().imageCounter_++;

    cv::Point2f c_temp;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      fManager.features_[ind].increaseStatistics(lastImageTime);
      mpCamera_->bearingToPixel(state.template get<mtState::_nor>(ind),c_temp);

      pixelOutputCF_.setIndex(ind);
      pixelOutputCov_ = pixelOutputCF_.transformCovMat(state,cov);
      fManager.features_[ind].log_prediction_.c_ = c_temp;
      fManager.features_[ind].log_prediction_.setSigmaFromCov(pixelOutputCov_);

      fManager.features_[ind].c_ = c_temp;
      fManager.features_[ind].currentStatistics_.inFrame_ = fManager.features_[ind].isMultilevelPatchInFrame(meas.template get<mtMeas::_aux>().pyr_,3);
      if(fManager.features_[ind].currentStatistics_.inFrame_) extractPixelCorner(state,ind);
      fManager.features_[ind].log_predictionC0_.c_ = fManager.features_[ind].c_ - 4*fManager.features_[ind].corners_[0] - 4*fManager.features_[ind].corners_[1];
      fManager.features_[ind].log_predictionC1_.c_ = fManager.features_[ind].c_ + 4*fManager.features_[ind].corners_[0] - 4*fManager.features_[ind].corners_[1];
      fManager.features_[ind].log_predictionC2_.c_ = fManager.features_[ind].c_ - 4*fManager.features_[ind].corners_[0] + 4*fManager.features_[ind].corners_[1];
      fManager.features_[ind].log_predictionC3_.c_ = fManager.features_[ind].c_ + 4*fManager.features_[ind].corners_[0] + 4*fManager.features_[ind].corners_[1];
    }
    const double t1 = (double) cv::getTickCount();
    int numSeq = startLevel_-endLevel_; // TODO adaptiv search (depending on covariance)
    fManager.alignFeaturesCom(meas.template get<mtMeas::_aux>().pyr_,state.template get<mtState::_aux>().img_,startLevel_,endLevel_,numSeq,doPatchWarping_);
    const double t2 = (double) cv::getTickCount();
    ROS_DEBUG_STREAM(" Matching " << fManager.validSet_.size() << " patches (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)");
    NormalVectorElement m_meas;
    Eigen::Vector2d vec2;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      if(fManager.features_[ind].currentStatistics_.status_ == TrackingStatistics::FOUND){
        fManager.features_[ind].log_meas_.c_ = fManager.features_[ind].c_;
        mpCamera_->pixelToBearing(fManager.features_[ind].c_,m_meas);
        m_meas.boxMinus(state.template get<mtState::_nor>(ind),vec2);
        if(useDirectMethod_ && (vec2.transpose()*cov.template block<2,2>(mtState::template getId<mtState::_nor>(ind),mtState::template getId<mtState::_nor>(ind)).inverse()*vec2)(0,0) > 5.886){ // TODO: param
          vec2.setZero();
        }
        state.difVecLin_.template block<2,1>(mtState::template getId<mtState::_nor>(ind),0) = vec2;
      } else if(useDirectMethod_ && fManager.features_[ind].currentStatistics_.inFrame_){
        state.difVecLin_.template block<2,1>(mtState::template getId<mtState::_nor>(ind),0).setZero();
      }
    }
  };
  void postProcess(mtState& state, mtCovMat& cov, const mtMeas& meas, mtOutlierDetection* mpOutlierDetection){
    FeatureManager<STATE::nLevels_,STATE::patchSize_,mtState::nMax_>& fManager = state.template get<mtState::_aux>().fManager_;
    MultilevelPatchFeature<STATE::nLevels_,STATE::patchSize_>* mpFeature;

    int countTracked = 0;
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      mpFeature = &fManager.features_[ind];
      mpCamera_->bearingToPixel(state.template get<mtState::_nor>(ind),mpFeature->log_current_.c_);
      pixelOutputCF_.setIndex(ind);
      pixelOutputCov_ = pixelOutputCF_.transformCovMat(state,cov);
      mpFeature->log_current_.setSigmaFromCov(pixelOutputCov_);

      // Status Handling
      if((useDirectMethod_ && fManager.features_[ind].currentStatistics_.inFrame_)
          || (!useDirectMethod_ && mpFeature->currentStatistics_.status_ == TrackingStatistics::FOUND)){
        if(!mpOutlierDetection->isOutlier(ind)){
          mpFeature->currentStatistics_.status_ = TrackingStatistics::TRACKED;
          countTracked++;
        } else {
          mpFeature->currentStatistics_.status_ = TrackingStatistics::OUTLIER;
        }
      }

      // Drawing
      if(fManager.features_[ind].currentStatistics_.inFrame_){
        mpFeature->log_prediction_.draw(state.template get<mtState::_aux>().img_,cv::Scalar(0,255,255));
//        mpFeature->log_predictionC0_.drawLine(state.template get<mtState::_aux>().img_,mpFeature->log_predictionC1_,cv::Scalar(0,255,255),1);
//        mpFeature->log_predictionC0_.drawLine(state.template get<mtState::_aux>().img_,mpFeature->log_predictionC2_,cv::Scalar(0,255,255),1);
//        mpFeature->log_predictionC3_.drawLine(state.template get<mtState::_aux>().img_,mpFeature->log_predictionC1_,cv::Scalar(0,255,255),1);
//        mpFeature->log_predictionC3_.drawLine(state.template get<mtState::_aux>().img_,mpFeature->log_predictionC2_,cv::Scalar(0,255,255),1);
        if(mpFeature->currentStatistics_.status_ == TrackingStatistics::TRACKED){
          mpFeature->log_current_.drawLine(state.template get<mtState::_aux>().img_,mpFeature->log_prediction_,cv::Scalar(0,255,0));
          mpFeature->log_current_.draw(state.template get<mtState::_aux>().img_,cv::Scalar(0, 255, 0));
          mpFeature->log_current_.drawText(state.template get<mtState::_aux>().img_,std::to_string(mpFeature->totCount_),cv::Scalar(0,255,0));
        } else if(mpFeature->currentStatistics_.status_ == TrackingStatistics::OUTLIER){
          mpFeature->log_current_.draw(state.template get<mtState::_aux>().img_,cv::Scalar(0, 0, 255));
          mpFeature->log_current_.drawText(state.template get<mtState::_aux>().img_,std::to_string(mpFeature->countStatistics(TrackingStatistics::OUTLIER,trackingLocalRange_)),cv::Scalar(0,0,255));
        } else {
          mpFeature->log_current_.drawText(state.template get<mtState::_aux>().img_,std::to_string(mpFeature->countStatistics(TrackingStatistics::NOTFOUND,trackingLocalVisibilityRange_)),cv::Scalar(0,255,255));
        }
      }
    }
    cv::rectangle(state.template get<mtState::_aux>().img_,cv::Point2f(0,0),cv::Point2f(82,92),cv::Scalar(50,50,50),-1,8,0);
    cv::rectangle(state.template get<mtState::_aux>().img_,cv::Point2f(0,0),cv::Point2f(80,90),cv::Scalar(100,100,100),-1,8,0);
    cv::putText(state.template get<mtState::_aux>().img_,std::to_string(state.template get<mtState::_aux>().imageCounter_),cv::Point2f(5,85),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,0));
    cv::Point2f rollCenter = cv::Point2f(40,40);
    cv::Scalar rollColor1(50,50,50);
    cv::Scalar rollColor2(200,200,200);
    cv::Scalar rollColor3(120,120,120);
    cv::circle(state.template get<mtState::_aux>().img_,rollCenter,32,rollColor1,-1,8,0);
    cv::circle(state.template get<mtState::_aux>().img_,rollCenter,30,rollColor2,-1,8,0);
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
    cv::fillPoly(state.template get<mtState::_aux>().img_,(const cv::Point**)&points,&nbtab,1,rollColor3);
    cv::line(state.template get<mtState::_aux>().img_,rollCenter+rollVector2,rollCenter+rollVector3,rollColor1, 2);
    cv::line(state.template get<mtState::_aux>().img_,rollCenter-rollVector2,rollCenter-rollVector3,rollColor1, 2);
    cv::ellipse(state.template get<mtState::_aux>().img_,rollCenter,cv::Size(10,10),0,0,180,rollColor1,2,8,0);
    cv::circle(state.template get<mtState::_aux>().img_,rollCenter,2,rollColor1,-1,8,0);

    // Extract feature patches
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      mpFeature = &fManager.features_[ind];
      if(mpFeature->currentStatistics_.status_ == TrackingStatistics::TRACKED){
        mpCamera_->bearingToPixel(state.template get<mtState::_nor>(ind),fManager.features_[ind].c_);
        fManager.features_[ind].currentStatistics_.inFrame_ = fManager.features_[ind].isMultilevelPatchWithBorderInFrame(meas.template get<mtMeas::_aux>().pyr_,3);
        if(fManager.features_[ind].currentStatistics_.inFrame_){
          mpFeature->extractPatchesFromImage(meas.template get<mtMeas::_aux>().pyr_);
          extractBearingCorner(state,ind);
        }
      }
    }

    // Remove bad feature
    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end();){
      const int ind = *it_f;
      ++it_f;
      if(!fManager.features_[ind].isGoodFeature(trackingLocalRange_,trackingLocalVisibilityRange_,trackingUpperBound_,trackingLowerBound_)
          || fManager.features_[ind].s_ < static_cast<float>(minAbsoluteSTScore_) + static_cast<float>(minRelativeSTScore_)*fManager.getAverageScore()){
        fManager.removeFeature(ind);
      }
    }

    // Check if enough free features
    int requiredFreeFeature = mtState::nMax_*minTrackedAndFreeFeatures_-countTracked;
    double factor = 1;
    auto it_f = fManager.validSet_.begin();
    while(static_cast<int>(fManager.invalidSet_.size()) < requiredFreeFeature){
      factor = factor*1.1;
      const int ind = *it_f;
      ++it_f;
      if(it_f == fManager.validSet_.end()){
        it_f = fManager.validSet_.begin();
      }
      if(fManager.features_[ind].currentStatistics_.status_ != TrackingStatistics::TRACKED && !fManager.features_[ind].isGoodFeature(trackingLocalRange_,trackingLocalVisibilityRange_,trackingUpperBound_*factor,trackingLowerBound_*factor)){ // TODO: improve
        fManager.removeFeature(ind);
      }
    }

    // Detect new feature if required
    fManager.candidates_.clear();
    if(fManager.validSet_.size() < startDetectionTh_*mtState::nMax_){
      ROS_DEBUG_STREAM(" Adding keypoints");
      const double t1 = (double) cv::getTickCount();
      const int detect_level = 1;
      std::vector<cv::Point2f> detected_keypoints;
      DetectFastCorners(meas.template get<mtMeas::_aux>().pyr_,detected_keypoints,detect_level);
      const double t2 = (double) cv::getTickCount();
      ROS_DEBUG_STREAM(" == Detected " << detected_keypoints.size() << " on level " << detect_level << " (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)");
      fManager.selectCandidates(detected_keypoints);
      const double t3 = (double) cv::getTickCount();
      ROS_DEBUG_STREAM(" == Selected " << fManager.candidates_.size() << " candidates (" << (t3-t2)/cv::getTickFrequency()*1000 << " ms)");
      fManager.extractCandidatePatchesFromImage(meas.template get<mtMeas::_aux>().pyr_);
      fManager.computeCandidatesScore(-1);
      const double t4 = (double) cv::getTickCount();
      ROS_DEBUG_STREAM(" == Extracting patches and computing scores of candidates (" << (t4-t3)/cv::getTickFrequency()*1000 << " ms)");
      std::unordered_set<unsigned int> newSet = fManager.addBestCandidates(mtState::nMax_-fManager.validSet_.size(),state.template get<mtState::_aux>().img_,
                                                                           nDetectionBuckets_, scoreDetectionExponent_, penaltyDistance_, zeroDistancePenalty_,false,static_cast<float>(minAbsoluteSTScore_) + static_cast<float>(minRelativeSTScore_)*fManager.getAverageScore());
      const double t5 = (double) cv::getTickCount();
      ROS_DEBUG_STREAM(" == Got " << fManager.validSet_.size() << " after adding (" << (t5-t4)/cv::getTickFrequency()*1000 << " ms)");
      for(auto it_f = newSet.begin();it_f != newSet.end(); ++it_f){
        const int ind = *it_f;
        V3D vec3;
        mpCamera_->pixelToBearing(fManager.features_[ind].c_,vec3);
        state.initializeFeatureState(cov,ind,vec3,initDepth_,initCovFeature_);
        extractBearingCorner(state,ind);
      }
    }

    for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){
      const int ind = *it_f;
      mpCamera_->bearingToPixel(state.template get<mtState::_nor>(ind),fManager.features_[ind].log_previous_.c_);
    }
  };
  bool extractBearingCorner(mtState& state, const int& ind) const{ // TODO: think about not in frame
    bool success = true;
    cv::Point2f pixelCorner;
    for(unsigned int i=0;i<2;i++){
      pixelCorner = state.template get<mtState::_aux>().fManager_.features_[ind].c_+state.template get<mtState::_aux>().fManager_.features_[ind].corners_[i];
      success = success & mpCamera_->pixelToBearing(pixelCorner,state.template get<mtState::_aux>().corners_[ind][i]);
    }
    return success;
  }
  bool extractPixelCorner(mtState& state, const int& ind) const{ // TODO: think about not in frame
    bool success = true;
    cv::Point2f pixelCorner;
    for(unsigned int i=0;i<2;i++){
      success = success & mpCamera_->bearingToPixel(state.template get<mtState::_aux>().corners_[ind][i],pixelCorner);
      state.template get<mtState::_aux>().fManager_.features_[ind].corners_[i] = pixelCorner - state.template get<mtState::_aux>().fManager_.features_[ind].c_;
    }
    return success;
  }
};

}


#endif /* IMGUPDATE_HPP_ */
