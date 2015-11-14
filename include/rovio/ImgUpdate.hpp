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

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/FilterStates.hpp"
#include "rovio/Camera.hpp"
#include "rovio/CoordinateTransform/PixelOutput.hpp"
#include "rovio/ZeroVelocityUpdate.hpp"
#include "rovio/MultilevelPatchAlignment.hpp"

namespace rovio {

/** \brief Class, defining the innovation.
 *
 *  @tparam STATE - Filter State
 */
template<typename STATE>
class ImgInnovation: public LWF::State<LWF::VectorElement<2>>{
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _pix = 0;
  ImgInnovation(){
    static_assert(_pix+1==E_,"Error with indices");
    this->template getName<_pix>() = "nor";
  };
  virtual ~ImgInnovation(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the image pyramids of the different cameras.
 *
 *  @tparam STATE - Filter State
 */
template<typename STATE>
class ImgUpdateMeasAuxiliary: public LWF::AuxiliaryBase<ImgUpdateMeasAuxiliary<STATE>>{
 public:
  ImgUpdateMeasAuxiliary(){
    reset(0.0);
  };
  virtual ~ImgUpdateMeasAuxiliary(){};
  void reset(const double t){
    imgTime_ = t;
    for(int i=0;i<STATE::nCam_;i++){
      isValidPyr_[i] = false;
    }
  }
  bool areAllValid(){
    for(int i=0;i<STATE::nCam_;i++){
      if(isValidPyr_[i] == false) return false;
    }
    return true;
  }
  ImagePyramid<STATE::nLevels_> pyr_[STATE::nCam_];
  bool isValidPyr_[STATE::nCam_];
  double imgTime_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief Update measurement class (all data in auxillary)
 *
 *  @tparam STATE - Filter State
 */
template<typename STATE>
class ImgUpdateMeas: public LWF::State<ImgUpdateMeasAuxiliary<STATE>>{
 public:
  typedef LWF::State<ImgUpdateMeasAuxiliary<STATE>> Base;
  using Base::E_;
  static constexpr unsigned int _aux = 0;
  ImgUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  virtual ~ImgUpdateMeas(){};

  //@{
  /** \brief Get the auxiliary state of the ImgUpdateMeas.
   *
   *  \see ImgUpdateMeasAuxiliary
   *  @return the the auxiliary state of the ImgUpdateMeas.
   */
  inline ImgUpdateMeasAuxiliary<STATE>& aux(){
    return this->template get<_aux>();
  }
  inline const ImgUpdateMeasAuxiliary<STATE>& aux() const{
    return this->template get<_aux>();
  }
  //@}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief Class holding the update noise.
 *
 *  @tparam STATE - Filter State
 */
template<typename STATE>
class ImgUpdateNoise: public LWF::State<LWF::VectorElement<2>>{
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _pix = 0;
  ImgUpdateNoise(){
    static_assert(_pix+1==E_,"Error with indices");
    this->template getName<_pix>() = "nor";
  };
  virtual ~ImgUpdateNoise(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Outlier Detection.
 *
 *  @tparam STATE - Filter State
 */
template<typename STATE>
class ImgOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_pix>(),2>>{
 public:
  /** \brief Destructor
   */
  virtual ~ImgOutlierDetection(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding image update routines for the filter.
 */
template<typename FILTERSTATE>
class ImgUpdate: public LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
ImgOutlierDetection<typename FILTERSTATE::mtState>,false>{
 public:
  typedef LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>,
      ImgOutlierDetection<typename FILTERSTATE::mtState>,false> Base;
  using Base::meas_;
  using Base::doubleRegister_;
  using Base::intRegister_;
  using Base::boolRegister_;
  using Base::updnoiP_;
  using Base::useSpecialLinearizationPoint_;
  using Base::useImprovedJacobian_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;

  // Multicamera pointer
  MultiCamera<mtState::nCam_>* mpMultiCamera_;

  // Parameter
  M3D initCovFeature_;
  double initDepth_;
  int startLevel_;
  int endLevel_;
  double startDetectionTh_;
  int nDetectionBuckets_;
  int fastDetectionThreshold_;
  double scoreDetectionExponent_;
  double penaltyDistance_;
  double zeroDistancePenalty_;
  double trackingUpperBound_,trackingLowerBound_;
  double minTrackedAndFreeFeatures_;
  double minRelativeSTScore_;
  double minAbsoluteSTScore_;
  double minTimeBetweenPatchUpdate_;
  double matchingPixelThreshold_;
  bool doVisualMotionDetection_; /**<Do visual motion detection*/
  double rateOfMovingFeaturesTh_; /**<What percentage of feature must be moving for image motion detection*/
  double pixelCoordinateMotionTh_; /**<Threshold for detecting feature motion*/
  int minFeatureCountForNoMotionDetection_; /**<Minimum amount of feature for detecting NO image motion*/
  double removalFactor_; /**<Factor for enforcing feature removal if not enough free*/
  double patchRejectionTh_;
  bool useDirectMethod_;  /**<If true, the innovation term is based directly on pixel intensity errors.
                              If false, the reprojection error is used for the innovation term.*/
  bool doFrameVisualisation_;
  bool visualizePatches_;
  bool verbose_;
  bool removeNegativeFeatureAfterUpdate_;
  double specialLinearizationThreshold_;
  bool isZeroVelocityUpdateEnabled_; /**<Should zero velocity updates be performed*/
  double minTimeForZeroVelocityUpdate_;  /**<Time until zero velocity update get performed if there is no motion*/
  ZeroVelocityUpdate<FILTERSTATE> zeroVelocityUpdate_; /**<Zero velocity update, directly integrated into the img update*/
  double maxUncertaintyToDepthRatioForDepthInitialization_;
  double bearingVectorMahalTh_; /**<Mahalnobis distance threshold of bearing error in search frame*/
  double updateNoiseNor_; /**<Update noise of update normal, used for inderect case (reprojection error)*/
  double updateNoiseInt_; /**<Update noise of intensity error, used for direct approach*/
  double alignConvergencePixelRange_;
  double alignCoverageRatio_;
  int alignMaxUniSample_;
  bool useCrossCameraMeasurements_; /**<Should features be matched across cameras.*/
  bool doStereoInitialization_; /**<Should a stereo match be used for feature initialization.*/
  int minNoAlignment_; /**<Minimal number of alignment every feature must make through.*/


  // Temporary
  mutable PixelOutputCT pixelOutputCT_;
  mutable PixelOutput pixelOutput_;
  mutable MXD pixelOutputCov_;
  mutable rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  mutable FeatureOutput featureOutput_;
  mutable MXD featureOutputCov_;
  mutable MXD featureOutputJac_;
  mutable MultilevelPatch<mtState::nLevels_,mtState::patchSize_> mlpTemp1_;
  mutable MultilevelPatch<mtState::nLevels_,mtState::patchSize_> mlpTemp2_;
  mutable FeatureCoordinates alignedCoordinates_;
  mutable FeatureCoordinates tempCoordinates_;
  mutable mtState linearizationPoint_;
  mutable MultilevelPatchAlignment<mtState::nLevels_,mtState::patchSize_> alignment_; /**<Patch aligner*/
  mutable std::vector<FeatureCoordinates> candidates_;
  mutable bool doPreAlignment_;

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
  ImgUpdate(): transformFeatureOutputCT_(nullptr), pixelOutputCov_((int)(PixelOutput::D_),(int)(PixelOutput::D_)), featureOutputCov_((int)(FeatureOutput::D_),(int)(FeatureOutput::D_)), featureOutputJac_((int)(FeatureOutput::D_),(int)(mtState::D_)){
    mpMultiCamera_ = nullptr;
    initCovFeature_.setIdentity();
    initDepth_ = 0.5;
    startLevel_ = 3;
    endLevel_ = 1;
    startDetectionTh_ = 0.9;
    nDetectionBuckets_ = 100;
    fastDetectionThreshold_ = 10;
    scoreDetectionExponent_ = 0.5;
    penaltyDistance_ = 20;
    zeroDistancePenalty_ = nDetectionBuckets_*1.0;
    useDirectMethod_ = true;
    doFrameVisualisation_ = true;
    visualizePatches_ = false;
    verbose_ = false;
    trackingUpperBound_ = 0.9;
    trackingLowerBound_ = 0.1;
    minTrackedAndFreeFeatures_ = 0.5;
    minRelativeSTScore_ = 0.2;
    minAbsoluteSTScore_ = 0.2;
    minTimeBetweenPatchUpdate_ = 1.0;
    matchingPixelThreshold_ = 4.0;
    patchRejectionTh_ = 10.0;
    removeNegativeFeatureAfterUpdate_ = true;
    specialLinearizationThreshold_ = 1.0/400;
    doVisualMotionDetection_ = false;
    rateOfMovingFeaturesTh_ = 0.5;
    pixelCoordinateMotionTh_ = 1.0;
    minFeatureCountForNoMotionDetection_ = 5;
    minTimeForZeroVelocityUpdate_ = 1.0;
    maxUncertaintyToDepthRatioForDepthInitialization_ = 0.3;
    bearingVectorMahalTh_ = 9.21;
    updateNoiseNor_ = 1e-5;
    updateNoiseInt_ = 4;
    alignConvergencePixelRange_ = 1.0;
    alignCoverageRatio_ = 2.0;
    alignMaxUniSample_ = 5;
    useCrossCameraMeasurements_ = true;
    doStereoInitialization_ = true;
    removalFactor_ = 1.1;
    minNoAlignment_ = 5;
    doubleRegister_.registerDiagonalMatrix("initCovFeature",initCovFeature_);
    doubleRegister_.registerScalar("initDepth",initDepth_);
    doubleRegister_.registerScalar("startDetectionTh",startDetectionTh_);
    doubleRegister_.registerScalar("scoreDetectionExponent",scoreDetectionExponent_);
    doubleRegister_.registerScalar("penaltyDistance",penaltyDistance_);
    doubleRegister_.registerScalar("zeroDistancePenalty",zeroDistancePenalty_);
    doubleRegister_.registerScalar("trackingUpperBound",trackingUpperBound_);
    doubleRegister_.registerScalar("trackingLowerBound",trackingLowerBound_);
    doubleRegister_.registerScalar("minTrackedAndFreeFeatures",minTrackedAndFreeFeatures_);
    doubleRegister_.registerScalar("minRelativeSTScore",minRelativeSTScore_);
    doubleRegister_.registerScalar("minAbsoluteSTScore",minAbsoluteSTScore_);
    doubleRegister_.registerScalar("minTimeBetweenPatchUpdate",minTimeBetweenPatchUpdate_);
    doubleRegister_.registerScalar("matchingPixelThreshold",matchingPixelThreshold_);
    doubleRegister_.registerScalar("patchRejectionTh",patchRejectionTh_);
    doubleRegister_.registerScalar("specialLinearizationThreshold",specialLinearizationThreshold_);
    doubleRegister_.registerScalar("MotionDetection.rateOfMovingFeaturesTh",rateOfMovingFeaturesTh_);
    doubleRegister_.registerScalar("MotionDetection.pixelCoordinateMotionTh",pixelCoordinateMotionTh_);
    doubleRegister_.registerScalar("maxUncertaintyToDepthRatioForDepthInitialization",maxUncertaintyToDepthRatioForDepthInitialization_);
    doubleRegister_.registerScalar("bearingVectorMahalTh",bearingVectorMahalTh_);
    doubleRegister_.registerScalar("alignConvergencePixelRange",alignConvergencePixelRange_);
    doubleRegister_.registerScalar("alignCoverageRatio",alignCoverageRatio_);
    doubleRegister_.registerScalar("removalFactor",removalFactor_);
    intRegister_.registerScalar("fastDetectionThreshold",fastDetectionThreshold_);
    intRegister_.registerScalar("startLevel",startLevel_);
    intRegister_.registerScalar("endLevel",endLevel_);
    intRegister_.registerScalar("nDetectionBuckets",nDetectionBuckets_);
    intRegister_.registerScalar("MotionDetection.minFeatureCountForNoMotionDetection",minFeatureCountForNoMotionDetection_);
    intRegister_.registerScalar("alignMaxUniSample",alignMaxUniSample_);
    intRegister_.registerScalar("minNoAlignment",minNoAlignment_);
    boolRegister_.registerScalar("MotionDetection.isEnabled",doVisualMotionDetection_);
    boolRegister_.registerScalar("useDirectMethod",useDirectMethod_);
    boolRegister_.registerScalar("doFrameVisualisation",doFrameVisualisation_);
    boolRegister_.registerScalar("visualizePatches",visualizePatches_);
    boolRegister_.registerScalar("removeNegativeFeatureAfterUpdate",removeNegativeFeatureAfterUpdate_);
    boolRegister_.registerScalar("useCrossCameraMeasurements",useCrossCameraMeasurements_);
    boolRegister_.registerScalar("doStereoInitialization",doStereoInitialization_);
    doubleRegister_.removeScalarByVar(updnoiP_(0,0));
    doubleRegister_.removeScalarByVar(updnoiP_(1,1));
    doubleRegister_.registerScalar("UpdateNoise.nor",updateNoiseNor_);
    doubleRegister_.registerScalar("UpdateNoise.int",updateNoiseInt_);
    useImprovedJacobian_ = false; // TODO: adapt/test
    isZeroVelocityUpdateEnabled_ = false;
    Base::PropertyHandler::registerSubHandler("ZeroVelocityUpdate",zeroVelocityUpdate_);
    zeroVelocityUpdate_.outlierDetection_.registerToPropertyHandler(&zeroVelocityUpdate_,"MahalanobisTh");
    zeroVelocityUpdate_.doubleRegister_.registerScalar("minNoMotionTime",minTimeForZeroVelocityUpdate_);
    zeroVelocityUpdate_.boolRegister_.registerScalar("isEnabled",isZeroVelocityUpdateEnabled_);
    intRegister_.removeScalarByStr("maxNumIteration");
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    doubleRegister_.removeScalarByStr("updateVecNormTermination");
  };

  /** \brief Destructor
   */
  virtual ~ImgUpdate(){};

  /** \brief Refresh the properties of the property handler
   */
  void refreshProperties(){
    useSpecialLinearizationPoint_ = true;
    if(isZeroVelocityUpdateEnabled_) assert(doVisualMotionDetection_);
    if(useDirectMethod_){
      updnoiP_.setIdentity();
      updnoiP_ = updnoiP_*updateNoiseInt_;
    } else {
      updnoiP_.setIdentity();
      updnoiP_ = updnoiP_*updateNoiseNor_;
    }
  };

  /** \brief Sets the multicamera pointer
   *
   * @param mpMultiCamera - Multicamera pointer
   */
  void setCamera(MultiCamera<mtState::nCam_>* mpMultiCamera){
    mpMultiCamera_ = mpMultiCamera;
    transformFeatureOutputCT_.mpMultiCamera_ = mpMultiCamera;
  }

  /** \brief Sets the innovation term.
   *
   *  \note If \ref useDirectMethod_ is set true, the innovation term is based directly on pixel intensity errors.
   *  \note If \ref useDirectMethod_ is set false, the reprojection error is used for the innovation term.
   *  @param mtInnovation - Class, holding innovation data.
   *  @param state        - Filter %State.
   *  @param noise        - Additive discrete Gaussian noise.
   */
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    const int& ID = state.aux().activeFeature_;  // Feature ID.
    const int& camID = state.CfP(ID).camID_;   // Camera ID of the feature.
    const int& activeCamCounter = state.aux().activeCameraCounter_;
    const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
    if(verbose_){
      std::cout << "    \033[32mMaking update with feature " << ID << " from camera " << camID << " in camera " << activeCamID << "\033[0m" << std::endl;
    }
    if(useDirectMethod_){
      y.template get<mtInnovation::_pix>() = state.aux().b_red_[ID]+noise.template get<mtNoise::_pix>();
    } else {
      transformFeatureOutputCT_.setFeatureID(ID);
      transformFeatureOutputCT_.setOutputCameraID(activeCamID);
      transformFeatureOutputCT_.transformState(state,featureOutput_);
      featureOutput_.c().get_nor().boxMinus(state.aux().bearingMeas_[ID],y.template get<mtInnovation::_pix>()); // 0 = m - m_meas + n
      y.template get<mtInnovation::_pix>() += noise.template get<mtNoise::_pix>();
    }
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  \note If \ref useDirectMethod_ is set true, the jacobian is set w.r.t. the intensity errors.
   *  \note If \ref useDirectMethod_ is set false, the jacobian is set w.r.t. the reprojection error.
   *  @param F     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   */
  void jacState(MXD& F, const mtState& state) const{
    const int& ID = state.aux().activeFeature_;
    const int& camID = state.CfP(ID).camID_;
    const int& activeCamCounter = state.aux().activeCameraCounter_;
    const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
    F.setZero();
    transformFeatureOutputCT_.setFeatureID(ID);
    transformFeatureOutputCT_.setOutputCameraID(activeCamID);
    transformFeatureOutputCT_.transformState(state,featureOutput_);
    transformFeatureOutputCT_.jacTransform(featureOutputJac_,state);
    if(useDirectMethod_){
      cv::Point2f c_temp;
      Eigen::Matrix2d c_J;
      mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(),c_temp,c_J);
      F = -state.aux().A_red_[ID]*c_J*featureOutputJac_.template block<2,mtState::D_>(0,0);
    } else {
      F = state.aux().bearingMeas_[ID].getN().transpose()
                *-LWF::NormalVectorElement::getRotationFromTwoNormalsJac(featureOutput_.c().get_nor(),state.aux().bearingMeas_[ID])
      *featureOutput_.c().get_nor().getM()*featureOutputJac_.template block<2,mtState::D_>(0,0);
    }
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  @param G     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   */
  void jacNoise(MXD& G, const mtState& state) const{
    G.setZero();
    G.template block<2,2>(mtInnovation::template getId<mtInnovation::_pix>(),mtNoise::template getId<mtNoise::_pix>()) = Eigen::Matrix2d::Identity();
  }

  /** \brief Prepares the filter state for the update.
   *
   *   @param filterState - Filter state.
   *   @param meas        - Update measurement.
   *   @todo sort feature by covariance and use more accurate ones first
   */
  void commonPreProcess(mtFilterState& filterState, const mtMeas& meas){
    assert(filterState.t_ == meas.aux().imgTime_);
    for(int i=0;i<mtState::nCam_;i++){
      if(doFrameVisualisation_){
        cvtColor(meas.aux().pyr_[i].imgs_[0], filterState.img_[i], CV_GRAY2RGB);
      }
    }
    filterState.imgTime_ = filterState.t_;
    filterState.imageCounter_++;
    filterState.patchDrawing_ = cv::Mat::zeros(mtState::nMax_*filterState.drawPS_,(1+2*mtState::nCam_)*filterState.drawPS_,CV_8UC3);
    filterState.state_.aux().activeFeature_ = 0;
    filterState.state_.aux().activeCameraCounter_ = 0;


    /* Detect Image changes by looking at the feature patches between current and previous image (both at the current feature location)
     * The maximum change of intensity is obtained if the pixel is moved along the strongest gradient.
     * The maximal singularvalue, which is equivalent to the root of the larger eigenvalue of the Hessian,
     * gives us range in which intensity change is allowed to be.
     */
    if(doVisualMotionDetection_ && filterState.imageCounter_>1){
      int totCountInFrame = 0;
      int totCountInMotion = 0;
      for(unsigned int i=0;i<mtState::nMax_;i++){
        if(filterState.fsm_.isValid_[i]){
          const int& camID = filterState.state_.CfP(i).camID_;   // Camera ID of the feature.
          tempCoordinates_ = *filterState.fsm_.features_[i].mpCoordinates_;
          tempCoordinates_.set_warp_identity();
          if(mlpTemp1_.isMultilevelPatchInFrame(filterState.prevPyr_[camID],tempCoordinates_,startLevel_,true)){
            mlpTemp1_.extractMultilevelPatchFromImage(filterState.prevPyr_[camID],tempCoordinates_,startLevel_,true);
            mlpTemp1_.computeMultilevelShiTomasiScore(endLevel_,startLevel_);
            mlpTemp2_.extractMultilevelPatchFromImage(meas.aux().pyr_[camID],tempCoordinates_,startLevel_,true);
            const float avgError = mlpTemp1_.computeAverageDifference(mlpTemp2_,endLevel_,startLevel_);
            if(avgError/std::sqrt(mlpTemp1_.e1_) > static_cast<float>(pixelCoordinateMotionTh_)) totCountInMotion++;
            totCountInFrame++;
          }
        }
      }
      if(rateOfMovingFeaturesTh_/totCountInMotion*totCountInFrame < 1.0 || totCountInFrame < minFeatureCountForNoMotionDetection_){
        filterState.state_.aux().timeSinceLastImageMotion_ = 0.0;
      }
    } else {
      filterState.state_.aux().timeSinceLastImageMotion_ = 0.0;
    }
  }

  /** \brief Pre-Processing for the image update.
   *
   *  Summary:
   *  1. Searches a valid MultilevelPatchFeature from the filter state.
   *  2. Transforms the MultilevelPatchFeature into the target frame.
   *  3. Executes a 2D patch alignment in the target frame. If unsuccessful go back to step 1.
   *  4. If bearing error between aligned patch and estimated patch too large, alter linearization point.
   *     Bearing vector in the state is directly altered to the new aligned position.
   *
   *  @param filterState - Filter state.
   *  @param meas        - Update measurement.
   *  @param isFinished  - True, if process has finished.
   *  @todo split into methods
   */
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished){
    if(isFinished){ // gets called if this is the first call
      commonPreProcess(filterState,meas);
      isFinished = false;
    }
    bool foundValidMeasurement = false;
    Eigen::Vector2d bearingError;
    typename mtFilterState::mtState& state = filterState.state_;
    MXD& cov = filterState.cov_;
    int& ID = filterState.state_.aux().activeFeature_;   // ID of the current updated feature!!! Initially set to 0.
    int& activeCamCounter = filterState.state_.aux().activeCameraCounter_;

    // Actualize camera extrinsics
    state.updateMultiCameraExtrinsics(mpMultiCamera_);

    while(ID < mtState::nMax_ && foundValidMeasurement == false){
      if(filterState.fsm_.isValid_[ID]){
        // Data handling stuff
        FeatureManager<mtState::nLevels_,mtState::patchSize_,mtState::nCam_>& f = filterState.fsm_.features_[ID];
        const int camID = f.mpCoordinates_->camID_;
        const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;
        if(activeCamCounter==0 && f.mpStatistics_->status_[activeCamID] != FAILED_TRACKING){
          f.mpStatistics_->increaseStatistics(filterState.t_);
          if(verbose_){
            std::cout << "=========== Feature " << f.idx_ << " ==================================================== " << std::endl;
          }
          // Visualize patch tracking
          if(visualizePatches_){
            f.mpMultilevelPatch_->drawMultilevelPatch(filterState.patchDrawing_,cv::Point2i(2,filterState.drawPB_+ID*filterState.drawPS_),1,false);
            cv::putText(filterState.patchDrawing_,std::to_string(f.idx_),cv::Point2i(2,10+ID*filterState.drawPS_),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255));
          }
        }
        if(verbose_){
          std::cout << "  ========== Camera  " << activeCamID << " ================= " << std::endl;
          std::cout << "  Normal in feature frame: " << f.mpCoordinates_->get_nor().getVec().transpose() << std::endl;
          std::cout << "  with depth: " << f.mpDistance_->getDistance() << std::endl;
        }

        // Get coordinates in target frame
        transformFeatureOutputCT_.setFeatureID(ID);
        transformFeatureOutputCT_.setOutputCameraID(activeCamID);
        transformFeatureOutputCT_.transformState(state,featureOutput_);
        transformFeatureOutputCT_.transformCovMat(state,cov,featureOutputCov_);
        if(verbose_) std::cout << "    Normal in camera frame: " << featureOutput_.c().get_nor().getVec().transpose() << std::endl;

        // Check if feature in target frame
        if(!featureOutput_.c().isInFront() || !mlpTemp1_.isMultilevelPatchInFrame(filterState.prevPyr_[camID],featureOutput_.c(),startLevel_,false)){
          f.mpStatistics_->status_[activeCamID] = NOT_IN_FRAME;
          if(verbose_) std::cout << "    NOT in frame" << std::endl;
        } else {
          pixelOutputCT_.transformState(featureOutput_,pixelOutput_);
          pixelOutputCT_.transformCovMat(featureOutput_,featureOutputCov_,pixelOutputCov_);
          featureOutput_.c().setPixelCov(pixelOutputCov_);

          // Visualization
          if(doFrameVisualisation_){
            if(activeCamID==camID){
              featureOutput_.c().drawEllipse(filterState.img_[activeCamID], cv::Scalar(0,175,175), 2.0, true);
              featureOutput_.c().drawText(filterState.img_[activeCamID],std::to_string(f.idx_),cv::Scalar(0,175,175));
            } else {
              featureOutput_.c().drawEllipse(filterState.img_[activeCamID], cv::Scalar(175,175,0), 2.0, true);
              featureOutput_.c().drawText(filterState.img_[activeCamID],std::to_string(f.idx_),cv::Scalar(175,175,0));
            }
          }
          if(visualizePatches_){
            if(mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID],featureOutput_.c(),mtState::nLevels_-1,false)){
              mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID],featureOutput_.c(),mtState::nLevels_-1,false);
              mlpTemp1_.drawMultilevelPatch(filterState.patchDrawing_,cv::Point2i(filterState.drawPB_+(1+2*activeCamID)*filterState.drawPS_,filterState.drawPB_+ID*filterState.drawPS_),1,false);
            }
          }
          if(activeCamID==camID){
            f.log_prediction_ = *f.mpCoordinates_;
          }

          // Do pre-alignment if required
          doPreAlignment_ = !useDirectMethod_ || featureOutput_.c().sigma1_ > matchingPixelThreshold_ || f.mpStatistics_->countTot() < minNoAlignment_ || f.mpStatistics_->status_[activeCamID] == FAILED_TRACKING;
          bool successfulPreAlignment = false;
          useSpecialLinearizationPoint_ = false;
          if(doPreAlignment_){
            if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+3,ID*filterState.drawPS_+3),3,cv::Scalar(0,255,0),-1,8,0);
            if(alignment_.align2DAdaptive(alignedCoordinates_,meas.aux().pyr_[activeCamID],*f.mpMultilevelPatch_,featureOutput_.c(),startLevel_,endLevel_,
                                          alignConvergencePixelRange_,alignCoverageRatio_,alignMaxUniSample_)){
              if(activeCamID==camID) f.log_meas_ = alignedCoordinates_;
              if(verbose_) std::cout << "    Found match: " << alignedCoordinates_.get_nor().getVec().transpose() << std::endl;
              if(mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID],alignedCoordinates_,startLevel_,false)){
                float avgError = 0.0;
                if(patchRejectionTh_ >= 0){
                  mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID],alignedCoordinates_,startLevel_,false);
                  avgError = mlpTemp1_.computeAverageDifference(*f.mpMultilevelPatch_,endLevel_,startLevel_);
                }
                if(patchRejectionTh_ >= 0 && avgError > patchRejectionTh_){
                  f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                  if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+9,ID*filterState.drawPS_+3),3,cv::Scalar(105,0,150),-1,8,0);
                  if(verbose_) std::cout << "    \033[31mREJECTED (error too large)\033[0m" << std::endl;
                } else {
                  alignedCoordinates_.get_nor().boxMinus(featureOutput_.c().get_nor(),bearingError);
                  const Eigen::Matrix2d bearingVectorCov = featureOutputCov_.template block<2,2>(FeatureOutput::template getId<FeatureOutput::_fea>(),FeatureOutput::template getId<FeatureOutput::_fea>()) +
                      Eigen::Matrix2d::Identity()*updateNoiseNor_;
                  const double weightedBearingError = bearingError.dot(bearingVectorCov.inverse()*bearingError);
                  if(weightedBearingError < bearingVectorMahalTh_){
                    if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+9,ID*filterState.drawPS_+3),3,cv::Scalar(0,255,0),-1,8,0);
                    if(doFrameVisualisation_) alignedCoordinates_.drawPoint(filterState.img_[activeCamID], cv::Scalar(255,0,255));
                    useSpecialLinearizationPoint_ = bearingError.norm() > specialLinearizationThreshold_;
                    if(verbose_) std::cout << "    useSpecialLinearizationPoint: " << useSpecialLinearizationPoint_ << std::endl;
                    if(useSpecialLinearizationPoint_) featureOutput_.c() = alignedCoordinates_;
                    successfulPreAlignment = true;
                  } else {
                    f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                    if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+9,ID*filterState.drawPS_+3),3,cv::Scalar(155,0,100),-1,8,0);
                    if(doFrameVisualisation_) alignedCoordinates_.drawPoint(filterState.img_[activeCamID], cv::Scalar(255,0,255));
                    if(verbose_) std::cout << "    \033[31mMatch too far! Mahalanobis distance: " << weightedBearingError << "\033[0m" << std::endl;
                  }
                }
              } else {
                f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+9,ID*filterState.drawPS_+3),3,cv::Scalar(55,0,200),-1,8,0);
                if(verbose_) std::cout << "    \033[31mNot in frame after alignment\033[0m" << std::endl;
              }
            } else {
              f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
              if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+9,ID*filterState.drawPS_+3),3,cv::Scalar(0,0,255),-1,8,0);
              if(verbose_) std::cout << "    \033[31mNOT FOUND (matching failed)\033[0m" << std::endl;
            }
          } else {
            if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+3,ID*filterState.drawPS_+3),3,cv::Scalar(0,0,255),-1,8,0);
            if(visualizePatches_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+9,ID*filterState.drawPS_+3),3,cv::Scalar(0,255,0),-1,8,0);
            if(verbose_) std::cout << "    Do direct update (without alignment)" << std::endl;
          }
          if(visualizePatches_){
            if(useSpecialLinearizationPoint_) cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+15,ID*filterState.drawPS_+3),3,cv::Scalar(0,255,0),-1,8,0);
            else cv::circle(filterState.patchDrawing_,cv::Point2i((1+2*activeCamID)*filterState.drawPS_+15,ID*filterState.drawPS_+3),3,cv::Scalar(0,0,255),-1,8,0);
          }

          // Build measurement
          if(!doPreAlignment_ || successfulPreAlignment){
            bool successfullBackProjection = false;
            if(useSpecialLinearizationPoint_){
              linearizationPoint_ = state;
              if(activeCamID == camID){
                linearizationPoint_.CfP(ID).set_nor(alignedCoordinates_.get_nor());
                successfullBackProjection = true;
              } else {
                Eigen::Matrix3d outputCov = Eigen::Matrix3d::Identity()*updateNoiseNor_;
                outputCov(2,2) = 1e6;
                successfullBackProjection = transformFeatureOutputCT_.solveInverseProblemRelaxed(linearizationPoint_,cov,featureOutput_,outputCov,1e-4,199); // TODO: make noide dependent on patch
              }
            }
            if(successfullBackProjection || !useSpecialLinearizationPoint_){
              if(verbose_ && useSpecialLinearizationPoint_) std::cout << "    Backprojection: " << linearizationPoint_.CfP(ID).get_nor().getVec().transpose() << std::endl;
              if(!useDirectMethod_ || alignment_.getLinearAlignEquationsReduced(meas.aux().pyr_[activeCamID],*f.mpMultilevelPatch_,featureOutput_.c()
                                                                                ,endLevel_,startLevel_,state.aux().A_red_[ID],state.aux().b_red_[ID])){
                if(useSpecialLinearizationPoint_) linearizationPoint_.boxMinus(state,filterState.difVecLin_);
                if(!useDirectMethod_) state.aux().bearingMeas_[ID] = alignedCoordinates_.get_nor();
                foundValidMeasurement = true;
                if(doFrameVisualisation_){
                  featureOutput_.c().drawPoint(filterState.img_[activeCamID], cv::Scalar(0,255,0));
                  if(activeCamID!=camID){
                    if(useSpecialLinearizationPoint_){
                      linearizationPoint_.CfP(ID).drawPoint(filterState.img_[camID], cv::Scalar(255,0,0));
                    }
                  }
                }
              } else {
                if(verbose_) std::cout << "    \033[31mFailed construction of linear equation!\033[0m" << std::endl;
              }
            } else {
              if(verbose_) std::cout << "    \033[31mFailed backprojection!\033[0m" << std::endl;
            }
            if(foundValidMeasurement == false){
              f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
            }
          }
        }
      }
      if(foundValidMeasurement == false){
        activeCamCounter++;
        if(activeCamCounter == mtState::nCam_ || !useCrossCameraMeasurements_){
          activeCamCounter = 0;
          ID++;
        }
      }
    }  // while end
    if(ID >= mtState::nMax_){
      isFinished = true;
    }
  };

  /** \brief Post-Processing for the image update.
   *
   *  Summary:
   *  1. Some drawing tasks.
   *  2. Removal of features with negative depth.
   *
   *  @param filterState      - Filter state.
   *  @param meas             - Update measurement.
   *  @param outlierDetection - Outlier detection.
   *  @param isFinished       - True, if process has finished.
   */
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    int& ID = filterState.state_.aux().activeFeature_;  // Get the ID of the updated feature.
    int& activeCamCounter = filterState.state_.aux().activeCameraCounter_;

    if(isFinished){
      commonPostProcess(filterState,meas);
    } else {
      FeatureManager<mtState::nLevels_,mtState::patchSize_,mtState::nCam_>& f = filterState.fsm_.features_[ID];
      const int camID = f.mpCoordinates_->camID_;
      const int activeCamID = (activeCamCounter + camID)%mtState::nCam_;

      // Remove negative feature
      if(removeNegativeFeatureAfterUpdate_){
        for(unsigned int i=0;i<mtState::nMax_;i++){
          if(filterState.fsm_.isValid_[i]){
            if(filterState.state_.dep(i).getDistance() < 1e-8){
              if(verbose_) std::cout << "    \033[33mRemoved feature " << filterState.fsm_.features_[i].idx_ << " with invalid distance parameter " << filterState.state_.dep(i).p_ << "!\033[0m" << std::endl;
              filterState.fsm_.isValid_[i] = false;
              filterState.resetFeatureCovariance(i,Eigen::Matrix3d::Identity());
            }
          }
        }
      }

      if(filterState.fsm_.isValid_[ID]){
        // Update statue and visualization
        if(activeCamID == camID){
          featureOutput_.c() = filterState.state_.CfP(ID);
        } else {
          transformFeatureOutputCT_.setFeatureID(ID);
          transformFeatureOutputCT_.setOutputCameraID(activeCamID);
          transformFeatureOutputCT_.transformState(filterState.state_,featureOutput_);
        }
        if(!outlierDetection.isOutlier(0)){
          if(mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[camID],featureOutput_.c(),startLevel_,false)){
            float avgError = 0.0;
            if(patchRejectionTh_ >= 0){
              mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[camID],featureOutput_.c(),startLevel_,false);
              avgError = mlpTemp1_.computeAverageDifference(*f.mpMultilevelPatch_,endLevel_,startLevel_);
            }
            if(patchRejectionTh_ < 0 || avgError <= patchRejectionTh_){
              f.mpStatistics_->status_[activeCamID] = TRACKED;
              if(doFrameVisualisation_) mlpTemp1_.drawMultilevelPatchBorder(filterState.img_[activeCamID],featureOutput_.c(),1.0,cv::Scalar(0,150+(activeCamID == camID)*105,0));
            } else {
              f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
              if(doFrameVisualisation_){
                mlpTemp1_.drawMultilevelPatchBorder(filterState.img_[activeCamID],featureOutput_.c(),1.0,cv::Scalar(0,0,150+(activeCamID == camID)*105));
                featureOutput_.c().drawText(filterState.img_[activeCamID],"PE: " + std::to_string(avgError),cv::Scalar(0,0,150+(activeCamID == camID)*105));
              }
              if(verbose_) std::cout << "    \033[31mToo large pixel error after update: " << avgError << "\033[0m" << std::endl;
            }
          } else {
            f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
            if(doFrameVisualisation_){
              mlpTemp1_.drawMultilevelPatchBorder(filterState.img_[activeCamID],featureOutput_.c(),1.0,cv::Scalar(0,0,150+(activeCamID == camID)*105));
              featureOutput_.c().drawText(filterState.img_[activeCamID],"NIF",cv::Scalar(0,0,150+(activeCamID == camID)*105));
            }
            if(verbose_) std::cout << "    \033[31mNot in frame after update!\033[0m" << std::endl;
          }
        } else {
          f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
          if(doFrameVisualisation_){
            mlpTemp1_.drawMultilevelPatchBorder(filterState.img_[activeCamID],featureOutput_.c(),1.0,cv::Scalar(0,0,150+(activeCamID == camID)*105));
            featureOutput_.c().drawText(filterState.img_[activeCamID],"MD: " + std::to_string(outlierDetection.getMahalDistance(0)),cv::Scalar(0,0,150+(activeCamID == camID)*105));
          }
          if(verbose_) std::cout << "    \033[31mRecognized as outlier by filter: " << outlierDetection.getMahalDistance(0) << "\033[0m" << std::endl;
        }

        // Visualize patch tracking
        if(visualizePatches_){
          if(mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID],featureOutput_.c(),mtState::nLevels_-1,false)){
            mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID],featureOutput_.c(),mtState::nLevels_-1,false);
            mlpTemp1_.drawMultilevelPatch(filterState.patchDrawing_,cv::Point2i(filterState.drawPB_+(2+2*activeCamID)*filterState.drawPS_,filterState.drawPB_+ID*filterState.drawPS_),1,false);
          }
          if(f.mpStatistics_->status_[activeCamID] == TRACKED){
            cv::rectangle(filterState.patchDrawing_,cv::Point2i((2+2*activeCamID)*filterState.drawPS_,ID*filterState.drawPS_),cv::Point2i((3+2*activeCamID)*filterState.drawPS_-1,(ID+1)*filterState.drawPS_-1),cv::Scalar(0,255,0),1,8,0);
          } else {
            cv::rectangle(filterState.patchDrawing_,cv::Point2i((2+2*activeCamID)*filterState.drawPS_,ID*filterState.drawPS_),cv::Point2i((3+2*activeCamID)*filterState.drawPS_-1,(ID+1)*filterState.drawPS_-1),cv::Scalar(0,0,255),1,8,0);
          }
        }
      }

      if(!doPreAlignment_ && f.mpStatistics_->status_[activeCamID] == FAILED_TRACKING){
        if(verbose_) std::cout << "    \033[33mDo second attempt with pre-alignment!\033[0m" << std::endl;
      } else {
        activeCamCounter++;
        if(activeCamCounter == mtState::nCam_ || !useCrossCameraMeasurements_){
          activeCamCounter = 0;
          ID++;
        }
      }
    }
  };

  /** \brief Final Post-Processing step for the image update.
   *
   *  Summary:
   *  1. For each feature in the state: Extract patches and compute Shi-Tomasi Score.
   *  2. Removal of bad features from the state.
   *  3. Get new features and add them to the state.
   *
   *  @param filterState      - Filter state.
   *  @param meas             - Update measurement.
   */
  void commonPostProcess(mtFilterState& filterState, const mtMeas& meas){
    // Temps
    float averageScore;
    int countTracked;
    int requiredFreeFeature;
    int featureIndex;
    typename mtFilterState::mtState& state = filterState.state_;
    MXD& cov = filterState.cov_;

    // Actualize camera extrinsics
    state.updateMultiCameraExtrinsics(mpMultiCamera_);

    countTracked = 0;
    // For all features in the state.
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.fsm_.isValid_[i]){
        FeatureManager<mtState::nLevels_,mtState::patchSize_,mtState::nCam_>& f = filterState.fsm_.features_[i];
        const int camID = f.mpCoordinates_->camID_;
        if(f.mpStatistics_->trackedInSomeFrame()){
          countTracked++;
        }
        if(f.mpStatistics_->status_[camID] == TRACKED && filterState.t_ - f.mpStatistics_->lastPatchUpdate_ > minTimeBetweenPatchUpdate_){
          tempCoordinates_ = *f.mpCoordinates_;
          tempCoordinates_.set_warp_identity();
          if(mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[camID],tempCoordinates_,startLevel_,true)){
            mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[camID],tempCoordinates_,startLevel_,true);
            mlpTemp1_.computeMultilevelShiTomasiScore(endLevel_,startLevel_);
            if(mlpTemp1_.s_ >= static_cast<float>(minAbsoluteSTScore_) && mlpTemp1_.s_ >= static_cast<float>(minRelativeSTScore_)*(f.mpMultilevelPatch_->s_)){
              *f.mpMultilevelPatch_ = mlpTemp1_;
              f.mpCoordinates_->set_warp_identity();
              f.mpStatistics_->lastPatchUpdate_ = filterState.t_;
            }
          }
        }
        // Visualize Quatlity
        if(visualizePatches_){
          for(int j=0;j<mtState::nCam_;j++){
            // Local Quality
            const double qLQ = f.mpStatistics_->getLocalQuality(j);
            cv::line(filterState.patchDrawing_,cv::Point2i((2+2*j)*filterState.drawPS_+1,(i+1)*filterState.drawPS_-4),cv::Point2i((2+2*j)*filterState.drawPS_+1+(filterState.drawPS_-3)*qLQ,(i+1)*filterState.drawPS_-4),cv::Scalar(0,255*qLQ,255*(1-qLQ)),2,8,0);
          }
          const double qALQ = f.mpStatistics_->getAverageLocalQuality();
          cv::line(filterState.patchDrawing_,cv::Point2i(1,(i+1)*filterState.drawPS_-10),cv::Point2i(1+(filterState.drawPS_-3)*qALQ,(i+1)*filterState.drawPS_-10),cv::Scalar(0,255*qALQ,255*(1-qALQ)),2,8,0);
          const double qLV = f.mpStatistics_->getLocalVisibility();
          cv::line(filterState.patchDrawing_,cv::Point2i(1,(i+1)*filterState.drawPS_-7),cv::Point2i(1+(filterState.drawPS_-3)*qLV,(i+1)*filterState.drawPS_-7),cv::Scalar(0,255*qLV,255*(1-qLV)),2,8,0);
          const double qGQ = f.mpStatistics_->getGlobalQuality();
          cv::line(filterState.patchDrawing_,cv::Point2i(1,(i+1)*filterState.drawPS_-4),cv::Point2i(1+(filterState.drawPS_-3)*qGQ,(i+1)*filterState.drawPS_-4),cv::Scalar(0,255*qGQ,255*(1-qALQ)),2,8,0);
        }
      }
    }

    // Remove bad feature.
    averageScore = filterState.fsm_.getAverageScore(); // TODO: make the following dependent on the ST-score
    if(verbose_) std::cout << "Removing features: ";
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.fsm_.isValid_[i]){
        FeatureManager<mtState::nLevels_,mtState::patchSize_,mtState::nCam_>& f = filterState.fsm_.features_[i];
        if(!f.mpStatistics_->isGoodFeature(trackingUpperBound_,trackingLowerBound_)){
          if(verbose_) std::cout << filterState.fsm_.features_[i].idx_ << ", ";
          filterState.fsm_.isValid_[i] = false;
          filterState.resetFeatureCovariance(i,Eigen::Matrix3d::Identity());
        }
      }
    }
    if(verbose_) std::cout << " | ";
    // Check if enough free features, enforce removal
    requiredFreeFeature = mtState::nMax_*minTrackedAndFreeFeatures_-countTracked;
    double factor = removalFactor_;
    featureIndex = 0;
    while((int)(mtState::nMax_) - (int)(filterState.fsm_.getValidCount()) < requiredFreeFeature){
      if(filterState.fsm_.isValid_[featureIndex]){
        FeatureManager<mtState::nLevels_,mtState::patchSize_,mtState::nCam_>& f = filterState.fsm_.features_[featureIndex];
        if(!f.mpStatistics_->trackedInSomeFrame() && !f.mpStatistics_->isGoodFeature(trackingUpperBound_*factor,trackingLowerBound_*factor)){
          if(verbose_) std::cout << filterState.fsm_.features_[featureIndex].idx_ << ", ";
          filterState.fsm_.isValid_[featureIndex] = false;
          filterState.resetFeatureCovariance(featureIndex,Eigen::Matrix3d::Identity());
        }
      }
      featureIndex++;
      if(featureIndex == mtState::nMax_){
        featureIndex = 0;
        factor = factor*removalFactor_;
      }
    }
    if(verbose_) std::cout << std::endl;

    // Get new features
    if(filterState.fsm_.getValidCount() < startDetectionTh_*mtState::nMax_){
      // Compute the median depth parameters for each camera, using the state features.
      std::array<double, mtState::nCam_> medianDepthParameters;
      if(maxUncertaintyToDepthRatioForDepthInitialization_>0){
        filterState.getMedianDepthParameters(initDepth_, &medianDepthParameters,maxUncertaintyToDepthRatioForDepthInitialization_);
      } else {
        medianDepthParameters.fill(initDepth_);
      }
      for(int camID = 0;camID<mtState::nCam_;camID++){
        // Get Candidates
        if(verbose_) std::cout << "Adding keypoints" << std::endl;
        const double t1 = (double) cv::getTickCount();
        candidates_.clear();
        for(int l=endLevel_;l<=startLevel_;l++){
          meas.aux().pyr_[camID].detectFastCorners(candidates_,l,fastDetectionThreshold_);
        }
        const double t2 = (double) cv::getTickCount();
        if(verbose_) std::cout << "== Detected " << candidates_.size() << " on levels " << endLevel_ << "-" << startLevel_ << " (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
        std::unordered_set<unsigned int> newSet = filterState.fsm_.addBestCandidates(candidates_,meas.aux().pyr_[camID],camID,filterState.t_,
                                                                    endLevel_,startLevel_,(mtState::nMax_-filterState.fsm_.getValidCount())/(mtState::nCam_-camID),nDetectionBuckets_, scoreDetectionExponent_,
                                                                    penaltyDistance_, zeroDistancePenalty_,false,minAbsoluteSTScore_);
        const double t3 = (double) cv::getTickCount();
        if(verbose_) std::cout << "== Got " << filterState.fsm_.getValidCount() << " after adding " << newSet.size() << " features in camera " << camID << " (" << (t3-t2)/cv::getTickFrequency()*1000 << " ms)" << std::endl;
        for(auto it = newSet.begin();it != newSet.end();++it){
          FeatureManager<mtState::nLevels_,mtState::patchSize_,mtState::nCam_>& f = filterState.fsm_.features_[*it];
          f.mpStatistics_->resetStatistics(filterState.t_);
          f.mpStatistics_->status_[camID] = TRACKED;
          f.mpStatistics_->lastPatchUpdate_ = filterState.t_;
          f.mpDistance_->p_ = medianDepthParameters[camID];
          filterState.resetFeatureCovariance(*it,initCovFeature_);
          if(doFrameVisualisation_){
            f.mpCoordinates_->drawPoint(filterState.img_[camID], cv::Scalar(255,0,0));
            f.mpCoordinates_->drawText(filterState.img_[camID],std::to_string(f.idx_),cv::Scalar(255,0,0));
          }

          if(mtState::nCam_>1 && doStereoInitialization_){
            const int otherCam = (camID+1)%mtState::nCam_;
            transformFeatureOutputCT_.setFeatureID(*it);
            transformFeatureOutputCT_.setOutputCameraID(otherCam);
            transformFeatureOutputCT_.transformState(filterState.state_,featureOutput_);
            if(alignment_.align2DAdaptive(alignedCoordinates_,meas.aux().pyr_[otherCam],*f.mpMultilevelPatch_,featureOutput_.c(),startLevel_,endLevel_,
                                            alignConvergencePixelRange_,alignCoverageRatio_,alignMaxUniSample_)){
              if(doFrameVisualisation_){
                alignedCoordinates_.drawPoint(filterState.img_[otherCam], cv::Scalar(150,0,0));
                alignedCoordinates_.drawText(filterState.img_[otherCam],std::to_string(f.idx_),cv::Scalar(150,0,0));
              }
              f.mpCoordinates_->getDepthFromTriangulation(alignedCoordinates_,state.qCM(otherCam).rotate(V3D(state.MrMC(otherCam)-state.MrMC(camID))),state.qCM(otherCam)*state.qCM(camID).inverted(), *f.mpDistance_);
            }
          }
        }
      }
    }
    for(unsigned int i=0;i<mtState::nMax_;i++){
      if(filterState.fsm_.isValid_[i]){
        filterState.fsm_.features_[i].log_previous_ = *filterState.fsm_.features_[i].mpCoordinates_;
      }
    }
    if (doFrameVisualisation_){
      for(int i=0;i<mtState::nCam_;i++){
        drawVirtualHorizon(filterState,i);
      }
    }

    if(verbose_){
      for(int i=0;i<mtState::nCam_;i++){
        std::cout << "Camera extrinsics: " << i << std::endl;
        std::cout << "  " << filterState.state_.qCM(i) << std::endl;
        std::cout << "  " << filterState.state_.MrMC(i).transpose() << std::endl;
      }
    }

    // Copy image pyramid to state
    for(int i=0;i<mtState::nCam_;i++){
      filterState.prevPyr_[i] = meas.aux().pyr_[i];
    }

    // Zero Velocity updates if appropriate
    if(isZeroVelocityUpdateEnabled_ && filterState.state_.aux().timeSinceLastImageMotion_ > minTimeForZeroVelocityUpdate_ && filterState.state_.aux().timeSinceLastInertialMotion_ > minTimeForZeroVelocityUpdate_){
      cv::putText(filterState.img_[0],"Performing Zero Velocity Updates!",cv::Point2f(150,25),cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255));
      zeroVelocityUpdate_.performUpdateEKF(filterState,ZeroVelocityUpdateMeas<mtState>());
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Draws a virtual horizon into the current image of the camera with ID camID
   *
   *  @param filterState - Filter state.
   *  @param camID       - ID of the camera, in which image the horizon should be drawn.
   */
  void drawVirtualHorizon(mtFilterState& filterState, const int camID = 0){
    typename mtFilterState::mtState& state = filterState.state_;
    cv::rectangle(filterState.img_[camID],cv::Point2f(0,0),cv::Point2f(82,92),cv::Scalar(50,50,50),-1,8,0);
    cv::rectangle(filterState.img_[camID],cv::Point2f(0,0),cv::Point2f(80,90),cv::Scalar(100,100,100),-1,8,0);
    cv::putText(filterState.img_[camID],std::to_string(filterState.imageCounter_),cv::Point2f(5,85),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,0));
    cv::Point2f rollCenter = cv::Point2f(40,40);
    cv::Scalar rollColor1(50,50,50);
    cv::Scalar rollColor2(200,200,200);
    cv::Scalar rollColor3(120,120,120);
    cv::circle(filterState.img_[camID],rollCenter,32,rollColor1,-1,8,0);
    cv::circle(filterState.img_[camID],rollCenter,30,rollColor2,-1,8,0);
    Eigen::Vector3d Vg = (state.qCM(camID)*state.qWM().inverted()).rotate(Eigen::Vector3d(0,0,-1));
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
    cv::fillPoly(filterState.img_[camID],(const cv::Point**)&points,&nbtab,1,rollColor3);
    cv::line(filterState.img_[camID],rollCenter+rollVector2,rollCenter+rollVector3,rollColor1, 2);
    cv::line(filterState.img_[camID],rollCenter-rollVector2,rollCenter-rollVector3,rollColor1, 2);
    cv::ellipse(filterState.img_[camID],rollCenter,cv::Size(10,10),0,0,180,rollColor1,2,8,0);
    cv::circle(filterState.img_[camID],rollCenter,2,rollColor1,-1,8,0);
  }
};

}


#endif /* ROVIO_IMGUPDATE_HPP_ */
