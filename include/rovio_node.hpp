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

#ifndef ROVIO_NODE_HPP_
#define ROVIO_NODE_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include "common_vision.hpp"
#include "Camera.hpp"

namespace rovio{


class TestNode{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg_;
  ros::Publisher pubPose_;
  static constexpr int n_levels_ = 4;
  cv::Mat draw_image_, img_;
  bool doVisualization;
  unsigned int min_feature_count_, max_feature_count_;
  ImagePyramid<n_levels_> pyr_;
  FeatureManager<n_levels_,8,100> fManager_;
  static constexpr int nDetectionBuckets_ = 100;
  static constexpr double scoreDetectionExponent_ = 0.5;
  static constexpr double penaltyDistance_ = 20;
  static constexpr double zeroDistancePenalty_ = nDetectionBuckets_*1.0;

  TestNode(ros::NodeHandle& nh): nh_(nh){
    subImu_ = nh_.subscribe("imuMeas", 1000, &TestNode::imuCallback,this);
    subImg_ = nh_.subscribe("/cam0/image_raw", 1000, &TestNode::imgCallback,this);
    doVisualization = true;
    min_feature_count_ = 50;
    max_feature_count_ = 20; // Maximal number of feature which is added at a time (not total)
    cv::namedWindow("test1");
    cv::namedWindow("test2");
    cv::namedWindow("test3");
    cv::namedWindow("Tracker");
  };
  ~TestNode(){
    cv::destroyWindow("test1");
    cv::destroyWindow("test2");
    cv::destroyWindow("test3");
    cv::destroyWindow("Tracker");
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
  }
  void imgCallback(const sensor_msgs::ImageConstPtr & img){
    // Get image from msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image.copyTo(img_);

    // Timing
    static double last_timeStamp = 0.0;
    double current_timeStamp = img->header.stamp.toSec();

    // Pyramid
    pyr_.computeFromImage(img_);

    // Drawing
    if (doVisualization)
      cv_ptr->image.copyTo(draw_image_);


    for(auto it_f = fManager_.validSet_.begin();it_f != fManager_.validSet_.end(); ++it_f){
      const int ind = *it_f;
      fManager_.features_[ind].increaseStatistics(last_timeStamp);
      fManager_.features_[ind].log_prediction_.c_ = fManager_.features_[ind].c_;
    }
    const double t1 = (double) cv::getTickCount();
    fManager_.alignFeaturesSeq(pyr_,draw_image_,3,1,false);
    const double t2 = (double) cv::getTickCount();
    ROS_INFO_STREAM(" Matching " << fManager_.validSet_.size() << " patches (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)");
    for(auto it_f = fManager_.validSet_.begin();it_f != fManager_.validSet_.end(); ++it_f){
      const int ind = *it_f;
      if(fManager_.features_[ind].currentStatistics_.status_ == TrackingStatistics::FOUND){
        fManager_.features_[ind].currentStatistics_.status_ = TrackingStatistics::TRACKED;
        fManager_.features_[ind].currentStatistics_.inFrame_ = true;
        fManager_.features_[ind].log_meas_.c_ = fManager_.features_[ind].c_;
        fManager_.features_[ind].log_meas_.draw(draw_image_,cv::Scalar(255,255,255));
        fManager_.features_[ind].log_meas_.drawLine(draw_image_,fManager_.features_[ind].log_prediction_,cv::Scalar(255,255,255));
        fManager_.features_[ind].log_meas_.drawText(draw_image_,std::to_string(fManager_.features_[ind].idx_),cv::Scalar(255,255,255));
      }

    }

    // Remove bad feature
    for(auto it_f = fManager_.validSet_.begin();it_f != fManager_.validSet_.end();){
      const int ind = *it_f;
      ++it_f;
      if(!fManager_.features_[ind].isGoodFeature()){ // TODO: handle inFrame
        fManager_.removeFeature(ind);
      }
    }

    // Extract feature patches
    for(auto it_f = fManager_.validSet_.begin();it_f != fManager_.validSet_.end(); ++it_f){
      const int ind = *it_f;
      if(fManager_.features_[ind].currentStatistics_.status_ == TrackingStatistics::TRACKED){
        fManager_.features_[ind].extractPatchesFromImage(pyr_);
      }
    }

    fManager_.candidates_.clear();
    if(fManager_.validSet_.size() < min_feature_count_){
      ROS_INFO_STREAM(" Adding keypoints");
      const double t1 = (double) cv::getTickCount();
      const int detect_level = 1;
      std::vector<cv::Point2f> detected_keypoints;
      DetectFastCorners(pyr_,detected_keypoints,detect_level);
      const double t2 = (double) cv::getTickCount();
      ROS_INFO_STREAM(" == Detected " << detected_keypoints.size() << " on level " << detect_level << " (" << (t2-t1)/cv::getTickFrequency()*1000 << " ms)");
      fManager_.selectCandidates(detected_keypoints);
      const double t3 = (double) cv::getTickCount();
      ROS_INFO_STREAM(" == Selected " << fManager_.candidates_.size() << " candidates (" << (t3-t2)/cv::getTickFrequency()*1000 << " ms)");
      fManager_.extractCandidatePatchesFromImage(pyr_);
      fManager_.computeCandidatesScore(-1);
      const double t4 = (double) cv::getTickCount();
      ROS_INFO_STREAM(" == Extracting patches and computing scores of candidates (" << (t4-t3)/cv::getTickFrequency()*1000 << " ms)");
      std::unordered_set<unsigned int> newSet = fManager_.addBestCandidates(max_feature_count_,draw_image_,
                                                                            nDetectionBuckets_, scoreDetectionExponent_, penaltyDistance_, zeroDistancePenalty_,true);
      const double t5 = (double) cv::getTickCount();
      ROS_INFO_STREAM(" == Got " << fManager_.validSet_.size() << " after adding (" << (t5-t4)/cv::getTickFrequency()*1000 << " ms)");
    }
    if (doVisualization) {
      cv::imshow("test1", pyr_.imgs_[1]);
      cv::imshow("test2", pyr_.imgs_[2]);
      cv::imshow("test3", pyr_.imgs_[3]);
      cv::imshow("Tracker", draw_image_);
      cv::waitKey(30);
    }
    last_timeStamp = current_timeStamp;
  }
};
}


#endif /* ROVIO_NODE_HPP_ */
