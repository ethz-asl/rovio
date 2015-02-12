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

#define EIGEN_STACK_ALLOCATION_LIMIT 1000000
#include <ros/ros.h>
#include <ros/package.h>
#include "rovio_filter.hpp"
#include <geometry_msgs/PoseStamped.h>


class TestFilter{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg_;
  ros::Publisher pubPose_;
  static constexpr unsigned int nMax_ = 50;
  static constexpr int n_levels_ = 4; // TODO: pull everywhere
  typedef rovio::FilterState<nMax_> mtState;
  typedef rovio::PredictionMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef rovio::ImgUpdateMeas<mtState> mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  rovio::Filter<mtState>* mpFilter;
  bool isInitialized_;
  geometry_msgs::PoseStamped poseMsg_;
  int poseMsgSeq_;

  TestFilter(ros::NodeHandle& nh): nh_(nh){
    #ifndef NDEBUG
      ROS_WARN("====================== Debug Mode ======================");
    #endif
    subImu_ = nh_.subscribe("/imu0", 1000, &TestFilter::imuCallback,this);
    subImg_ = nh_.subscribe("/cam0/image_raw", 1000, &TestFilter::imgCallback,this);
    pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/rovio/pose", 1);
    mpFilter = new rovio::Filter<mtState>();
    std::string rootdir = ros::package::getPath("rovio");
    mpFilter->readFromInfo(rootdir + "/cfg/rovio.info");
    poseMsg_.header.frame_id = "/world";
    poseMsgSeq_ = 1;
    cv::namedWindow("Tracker");
    isInitialized_ = false;

    bool makeTest = true;
    if(makeTest){
      mtState testState;
      testState.setRandom(1);
      rovio::MultilevelPatchFeature<n_levels_,8> feature;
      unsigned int s = 1;
      LWF::QuaternionElement q;
      LWF::VectorElement<3> vec;
      q.setRandom(s);
      vec.setRandom(s);
      mpFilter->mPrediction_.qVM_ = q.q_;
      mpFilter->mPrediction_.MrMV_ = vec.v_;
      for(unsigned int i=0;i<nMax_;i++){
        testState.template get<mtState::_aux>().fManager_.addFeature(feature);
        testState.template get<mtState::_aux>().fManager_.features_[i].inFrame_ = true;
        testState.template get<mtState::_aux>().fManager_.features_[i].trackingSuccess_ = true;
        testState.template get<mtState::_aux>().fManager_.features_[i].c_ = cv::Point2f((i*39829)%250,(i*49922)%250);
      }
      testState.template get<mtState::_aux>().fManager_.features_[2].inFrame_ = false;
      testState.template get<mtState::_aux>().fManager_.features_[1].trackingSuccess_ = false;
      testState.template get<mtState::_aux>().fManager_.removeFeature(0);
      predictionMeas_.setRandom(1);
      imgUpdateMeas_.setRandom(1);

      mpFilter->mPrediction_.testJacs(testState,predictionMeas_,1e-8,1e-6,0,0.1);
      std::get<0>(mpFilter->mUpdates_).testJacs(testState,imgUpdateMeas_,1e-9,1e-6,0,0.1);
    }
  }
  ~TestFilter(){
    cv::destroyWindow("Tracker");
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    predictionMeas_.template get<mtPredictionMeas::_acc>() = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
    if(isInitialized_){
      mpFilter->addPredictionMeas(predictionMeas_,imu_msg->header.stamp.toSec());
    } else {
      mpFilter->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),imu_msg->header.stamp.toSec());
      std::cout << "-- Filter: Initialized!" << std::endl;
      isInitialized_ = true;
    }
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
    cv::Mat cv_img;
    cv_ptr->image.copyTo(cv_img);

    if(isInitialized_ && !cv_img.empty()){
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_.computeFromImage(cv_img);
      mpFilter->addUpdateMeas<0>(imgUpdateMeas_,img->header.stamp.toSec()-0.05); // TODO

      mpFilter->updateSafe();
      std::cout << mpFilter->safe_.state_.template get<mtState::_vep>().transpose() << std::endl;
      std::cout << mpFilter->safe_.state_.template get<mtState::_vea>() << std::endl;
      if(!mpFilter->safe_.state_.template get<mtState::_aux>().img_.empty()){
        cv::imshow("Tracker", mpFilter->safe_.state_.template get<mtState::_aux>().img_);
        cv::waitKey(30);
      }
      if(pubPose_.getNumSubscribers() > 0){
        poseMsg_.header.seq = poseMsgSeq_++;
        poseMsg_.header.stamp = ros::Time(mpFilter->safe_.t_);

        poseMsg_.pose.position.x = mpFilter->safe_.state_.template get<mtState::_pos>()(0);
        poseMsg_.pose.position.y = mpFilter->safe_.state_.template get<mtState::_pos>()(1);
        poseMsg_.pose.position.z = mpFilter->safe_.state_.template get<mtState::_pos>()(2);
        poseMsg_.pose.orientation.w = mpFilter->safe_.state_.template get<mtState::_att>().w();
        poseMsg_.pose.orientation.x = -mpFilter->safe_.state_.template get<mtState::_att>().x();
        poseMsg_.pose.orientation.y = -mpFilter->safe_.state_.template get<mtState::_att>().y();
        poseMsg_.pose.orientation.z = -mpFilter->safe_.state_.template get<mtState::_att>().z();
        pubPose_.publish(poseMsg_);
      }
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "TestFilter");
  ros::NodeHandle nh;
  TestFilter testFilter(nh);
  ros::spin();
  return 0;
}
