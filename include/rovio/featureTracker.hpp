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

#ifndef FEATURE_TRACKER_HPP_
#define FEATURE_TRACKER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include "rovio/MultiCamera.hpp"
#include "rovio/FeatureManager.hpp"
#include "rovio/MultilevelPatchAlignment.hpp"

namespace rovio{

/** \brief Ros Node, executing a MultilevelPatchFeature tracking on an incoming image stream.
 */
class FeatureTrackerNode{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;  /**<IMU subscriber.*/
  ros::Subscriber subImg_;  /**<Image subscriber.*/
  static constexpr int nMax_ = 100;  /**<Maximum number of MultilevelPatchFeature%s in a MultilevelPatchSet.*/
  static constexpr int patchSize_ = 8;  /**<Edge length of the patches in pixels. Value must be a multiple of 2!*/
  static constexpr int nLevels_ = 4;  /**<Total number of image pyramid levels.*/
  static constexpr int nCam_ = 1;  /**<Total number of image pyramid levels. Only 1 camera supported so far.*/
  cv::Mat draw_image_, img_, draw_patches_;
  unsigned int min_feature_count_;  /**<New MultilevelPatchFeature%s are added to the existing MultilevelPatchSet,
                                        if the number of valid MultilevelPatchFeature%s in the set is smaller
                                        than min_feature_count_.*/
  unsigned int max_feature_count_;  /**<Maximal number of features, which are added at a time (not total). See rovio::addBestCandidates().*/
  ImagePyramid<nLevels_> pyr_;
  FeatureSetManager<nLevels_,patchSize_,nCam_,nMax_> fsm_;
  MultilevelPatchAlignment<nLevels_,patchSize_> alignment_;
  static constexpr int nDetectionBuckets_ = 100;  /**<See rovio::addBestCandidates().*/
  static constexpr double scoreDetectionExponent_ = 0.25;  /**<See rovio::addBestCandidates().*/
  static constexpr double penaltyDistance_ = 20;  /**<See rovio::addBestCandidates().*/
  static constexpr double zeroDistancePenalty_ = nDetectionBuckets_*100.0;  /**<See rovio::addBestCandidates(). [nDetectionBuckets_*100] is a strong penalty, thus features with a distance of less penaltyDistance_ will not be added.*/
  static constexpr int l1 = 1; /**<Minimal pyramid level, which should be used e.g. for corner detection and patch alignment. (l1<l2)*/
  static constexpr int l2 = 3; /**<Maximal pyramid level, which should be used e.g. for corner detection and patch alignment. (l1<l2)*/
  static constexpr int detectionThreshold = 10; /**<See rovio::detectFastCorners().*/
  static constexpr bool drawNotFound_ = false;  /**<Draw MultilevelPatchFeature%s which were not found again.*/
  rovio::MultiCamera<nCam_> multiCamera_;

  /** \brief Constructor
   */
  FeatureTrackerNode(ros::NodeHandle& nh): nh_(nh), fsm_(&multiCamera_){
    static_assert(l2>=l1, "l2 must be larger than l1");
    subImu_ = nh_.subscribe("imuMeas", 1000, &FeatureTrackerNode::imuCallback,this);
    subImg_ = nh_.subscribe("/cam0/image_raw", 1000, &FeatureTrackerNode::imgCallback,this);
    min_feature_count_ = 50;
    max_feature_count_ = 20; // Maximal number of feature which is added at a time (not total)
    cv::namedWindow("Tracker");
    multiCamera_.cameras_[0].load("/home/michael/calibrations/p22035_equidist.yaml");
    fsm_.allocateMissing();
  };

  /** \brief Destructor.
   */
  virtual ~FeatureTrackerNode(){}

  /** \brief Empty, yet.
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
  }

  /** \brief Image callback, handling the tracking of MultilevelPatchFeature%s.
   *
   *  The sequence of the callback can be summarized as follows:
   *  1. Extract image from message. Compute the image pyramid from the extracted image.
   *  2. Predict the position of the valid MultilevelPatchFeature%s in the current image,
   *     using the previous 2 image locations of these MultilevelPatchFeature%s.
   *  3. Execute 2D patch alignment at the predicted MultilevelPatchFeature locations.
   *     If successful the matching status of the MultilevelPatchFeature is set to FOUND and its image location is updated.
   *  4. Prune: Check the MultilevelPatchFeature%s in the MultilevelPatchSet for their quality (MultilevelPatchFeature::isGoodFeature()).
   *            If a bad quality of a MultilevelPatchFeature is recognized, it is set to invalid.
   *  5. Get new features and add them to the MultilevelPatchSet, if there are too little valid MultilevelPatchFeature%s
   *     in the MultilevelPatchSet. MultilevelPatchFeature%s which are stated invalid are replaced by new features.
   *
   *  @param img_msg - Image message (ros)
   */
  void imgCallback(const sensor_msgs::ImageConstPtr & img_msg){
    // Get image from msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image.copyTo(img_);

    // Timing
    static double last_time = 0.0;
    double current_time = img_msg->header.stamp.toSec();

    // Pyramid
    pyr_.computeFromImage(img_,true);

    // Drawing
    cvtColor(img_, draw_image_, CV_GRAY2RGB);
    const int numPatchesPlot = 10;
    draw_patches_ = cv::Mat::zeros(numPatchesPlot*(patchSize_*pow(2,nLevels_-1)+4),3*(patchSize_*pow(2,nLevels_-1)+4),CV_8UC1);

    // Prediction
    cv::Point2f dc;
    for(unsigned int i=0;i<nMax_;i++){
      if(fsm_.isValid_[i]){
        dc = 0.75*(fsm_.features_[i].mpCoordinates_->get_c() - fsm_.features_[i].log_previous_.get_c());
        fsm_.features_[i].log_previous_ = *(fsm_.features_[i].mpCoordinates_);
        fsm_.features_[i].mpCoordinates_->set_c(fsm_.features_[i].mpCoordinates_->get_c() + dc);
        if(!fsm_.features_[i].mpMultilevelPatch_->isMultilevelPatchInFrame(pyr_,*(fsm_.features_[i].mpCoordinates_),nLevels_-1,false)){
          fsm_.features_[i].mpCoordinates_->set_c(fsm_.features_[i].log_previous_.get_c());
        }
        fsm_.features_[i].mpStatistics_->increaseStatistics(current_time);
        for(int j=0;j<nCam_;j++){
          fsm_.features_[i].mpStatistics_->status_[j] = UNKNOWN;
        }
      }
    }

    // Track valid features
    FeatureCoordinates alignedCoordinates;
    const double t1 = (double) cv::getTickCount();
    for(unsigned int i=0;i<nMax_;i++){
      if(fsm_.isValid_[i]){
        fsm_.features_[i].log_prediction_ = *(fsm_.features_[i].mpCoordinates_);
        if(alignment_.align2DComposed(alignedCoordinates,pyr_,*fsm_.features_[i].mpMultilevelPatch_,*fsm_.features_[i].mpCoordinates_,l2,l1,l1)){
          fsm_.features_[i].mpStatistics_->status_[0] = TRACKED;
          fsm_.features_[i].mpCoordinates_->set_c(alignedCoordinates.get_c());
          fsm_.features_[i].log_previous_ = *(fsm_.features_[i].mpCoordinates_);
          fsm_.features_[i].mpCoordinates_->drawPoint(draw_image_,cv::Scalar(0,255,255));
          fsm_.features_[i].mpCoordinates_->drawLine(draw_image_,fsm_.features_[i].log_prediction_,cv::Scalar(0,255,255));
          fsm_.features_[i].mpCoordinates_->drawText(draw_image_,std::to_string(i),cv::Scalar(0,255,255));
          if(i==18){
            for(int j=0;j<nLevels_;j++){
//              std::cout << fsm_.features_[i].mpMultilevelPatch_->isValidPatch_[j] << std::endl;
//              std::cout << fsm_.features_[i].mpMultilevelPatch_->patches_[j].dx_[0] << std::endl;
            }
//            std::cout << alignment_.A_.transpose() << std::endl;
//            std::cout << alignment_.b_.transpose() << std::endl;
          }

        } else {
          fsm_.features_[i].mpStatistics_->status_[0] = FAILED_ALIGNEMENT;
          fsm_.features_[i].mpCoordinates_->drawPoint(draw_image_,cv::Scalar(0,0,255));
          fsm_.features_[i].mpCoordinates_->drawText(draw_image_,std::to_string(fsm_.features_[i].idx_),cv::Scalar(0,0,255));
        }
      }
    }
    const double t2 = (double) cv::getTickCount();
    ROS_INFO_STREAM(" Matching " << fsm_.getValidCount() << " patches (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)");
    MultilevelPatch<nLevels_,patchSize_> mp;
    for(unsigned int i=0;i<numPatchesPlot;i++){
      if(fsm_.isValid_[i+10]){
        fsm_.features_[i+10].mpMultilevelPatch_->drawMultilevelPatch(draw_patches_,cv::Point2i(2,2+i*(patchSize_*pow(2,nLevels_-1)+4)),1,false);
        if(mp.isMultilevelPatchInFrame(pyr_,fsm_.features_[i+10].log_prediction_,nLevels_-1,false)){
          mp.extractMultilevelPatchFromImage(pyr_,fsm_.features_[i+10].log_prediction_,nLevels_-1,false);
          mp.drawMultilevelPatch(draw_patches_,cv::Point2i(patchSize_*pow(2,nLevels_-1)+6,2+i*(patchSize_*pow(2,nLevels_-1)+4)),1,false);
        }
        if(fsm_.features_[i+10].mpStatistics_->status_[0] == TRACKED
            && mp.isMultilevelPatchInFrame(pyr_,*fsm_.features_[i+10].mpCoordinates_,nLevels_-1,false)){
          mp.extractMultilevelPatchFromImage(pyr_,*fsm_.features_[i+10].mpCoordinates_,nLevels_-1,false);
          mp.drawMultilevelPatch(draw_patches_,cv::Point2i(2*patchSize_*pow(2,nLevels_-1)+10,2+i*(patchSize_*pow(2,nLevels_-1)+4)),1,false);
          cv::rectangle(draw_patches_,cv::Point2i(0,i*(patchSize_*pow(2,nLevels_-1)+4)),cv::Point2i(patchSize_*pow(2,nLevels_-1)+3,(i+1)*(patchSize_*pow(2,nLevels_-1)+4)-1),cv::Scalar(255),2,8,0);
          cv::rectangle(draw_patches_,cv::Point2i(patchSize_*pow(2,nLevels_-1)+4,i*(patchSize_*pow(2,nLevels_-1)+4)),cv::Point2i(2*patchSize_*pow(2,nLevels_-1)+7,(i+1)*(patchSize_*pow(2,nLevels_-1)+4)-1),cv::Scalar(255),2,8,0);
        } else {
          cv::rectangle(draw_patches_,cv::Point2i(0,i*(patchSize_*pow(2,nLevels_-1)+4)),cv::Point2i(patchSize_*pow(2,nLevels_-1)+3,(i+1)*(patchSize_*pow(2,nLevels_-1)+4)-1),cv::Scalar(0),2,8,0);
          cv::rectangle(draw_patches_,cv::Point2i(patchSize_*pow(2,nLevels_-1)+4,i*(patchSize_*pow(2,nLevels_-1)+4)),cv::Point2i(2*patchSize_*pow(2,nLevels_-1)+7,(i+1)*(patchSize_*pow(2,nLevels_-1)+4)-1),cv::Scalar(0),2,8,0);
        }
        cv::putText(draw_patches_,std::to_string(fsm_.features_[i+10].idx_),cv::Point2i(2,2+i*(patchSize_*pow(2,nLevels_-1)+4)+10),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
      }
    }

    // Prune
    // Check the MultilevelPatchFeature%s in the MultilevelPatchSet for their quality (isGoodFeature(...)).
    // If a bad quality of a MultilevelPatchFeature is recognized, it is set to invalid. New MultilevelPatchFeature%s
    // replace the array places of invalid MultilevelPatchFeature%s in the MultilevelPatchSet.
    int prune_count = 0;
    for(unsigned int i=0;i<nMax_;i++){
      if(fsm_.isValid_[i]){
        if(fsm_.features_[i].mpStatistics_->status_[0] == FAILED_ALIGNEMENT){
          fsm_.isValid_[i] = false;
          prune_count++;
        }
      }
    }
    ROS_INFO_STREAM(" Pruned " << prune_count << " features");

    // Extract feature patches
    // Extract new MultilevelPatchFeature%s at the current tracked feature positions.
    // Extracted multilevel patches are aligned with the image axes.
    for(unsigned int i=0;i<nMax_;i++){
      if(fsm_.isValid_[i]){
        if(fsm_.features_[i].mpStatistics_->status_[0] == TRACKED
            && fsm_.features_[i].mpMultilevelPatch_->isMultilevelPatchInFrame(pyr_,*fsm_.features_[i].mpCoordinates_,nLevels_-1,true)){
          fsm_.features_[i].mpMultilevelPatch_->extractMultilevelPatchFromImage(pyr_,*fsm_.features_[i].mpCoordinates_,nLevels_-1,true);
        }
      }
    }

    // Get new features, if there are too little valid MultilevelPatchFeature%s in the MultilevelPatchSet.
    if(fsm_.getValidCount() < min_feature_count_){
      FeatureCoordinatesVec candidates;
      ROS_INFO_STREAM(" Adding keypoints");
      const double t1 = (double) cv::getTickCount();
      for(int l=l1;l<=l2;l++){
        pyr_.detectFastCorners(candidates,l,detectionThreshold);
      }
      const double t2 = (double) cv::getTickCount();
      ROS_INFO_STREAM(" == Detected " << candidates.size() << " on levels " << l1 << "-" << l2 << " (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)");
//      pruneCandidates(fsm_,candidates,0);
      const double t3 = (double) cv::getTickCount();
//      ROS_INFO_STREAM(" == Selected " << candidates.size() << " candidates (" << (t3-t2)/cv::getTickFrequency()*1000 << " ms)");
      std::unordered_set<unsigned int> newSet = fsm_.addBestCandidates( candidates,pyr_,0,current_time,
                                                                        l1,l2,max_feature_count_,nDetectionBuckets_, scoreDetectionExponent_,
                                                                        penaltyDistance_, zeroDistancePenalty_,true,0.0);
      const double t4 = (double) cv::getTickCount();
      ROS_INFO_STREAM(" == Got " << fsm_.getValidCount() << " after adding " << newSet.size() << " features (" << (t4-t3)/cv::getTickFrequency()*1000 << " ms)");
      for(auto it = newSet.begin();it != newSet.end();++it){
        fsm_.features_[*it].log_previous_ = *(fsm_.features_[*it].mpCoordinates_);
        for(int j=0;j<nCam_;j++){
          fsm_.features_[*it].mpStatistics_->status_[j] = TRACKED;
        }
      }
    }

    cv::imshow("Tracker", draw_image_);
    cv::imshow("Patches", draw_patches_);
    cv::waitKey(30);
    last_time = current_time;
  }
};
}


#endif /* FEATURE_TRACKER_HPP_ */
