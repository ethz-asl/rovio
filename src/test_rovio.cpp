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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "CameraOutputCF.hpp"


class TestFilter{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg_;
  ros::Publisher pubPose_;
  static constexpr unsigned int nMax_ = 50;
  static constexpr int nLevels_ = 4;
  static constexpr int patchSize_ = 8;
  typedef rovio::FilterState<nMax_,nLevels_,patchSize_> mtState;
  typedef rovio::PredictionMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef rovio::ImgUpdateMeas<mtState> mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  typedef rovio::Filter<mtState> mtFilter;
  mtFilter* mpFilter;
  bool isInitialized_;
  geometry_msgs::PoseStamped poseMsg_;
  int poseMsgSeq_;
  typedef rovio::StandardOutput mtOutput;
  mtOutput output_;
  mtOutput::mtCovMat outputCov_;
  rovio::CameraOutputCF<typename mtFilter::mtPrediction> cameraOutputCF_;

  TestFilter(ros::NodeHandle& nh): nh_(nh), mpFilter(new rovio::Filter<mtState>()), cameraOutputCF_(mpFilter->mPrediction_){
    #ifndef NDEBUG
      ROS_WARN("====================== Debug Mode ======================");
    #endif
    subImu_ = nh_.subscribe("/imu0", 1000, &TestFilter::imuCallback,this);
    subImg_ = nh_.subscribe("/cam0/image_raw", 1000, &TestFilter::imgCallback,this);
    pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/rovio/pose", 1);
    std::string rootdir = ros::package::getPath("rovio");
    mpFilter->readFromInfo(rootdir + "/cfg/rovio.info");
    poseMsg_.header.frame_id = "/world";
    poseMsgSeq_ = 1;
    cv::namedWindow("Tracker");
    isInitialized_ = false;

    bool makeTest = true;
    if(makeTest){
      mtState testState;
      unsigned int s = 2;
      testState.setRandom(s); // TODO: debug with   doVECalibration = false and depthType = 0
      rovio::MultilevelPatchFeature<nLevels_,patchSize_> feature;
      LWF::QuaternionElement q;
      LWF::VectorElement<3> vec;
      LWF::QuaternionElement q_temp;
      LWF::VectorElement<3> vec_temp;
      q.setRandom(s);
      vec.setRandom(s);
      q_temp.q_ = mpFilter->mPrediction_.qVM_;
      vec_temp.v_ = mpFilter->mPrediction_.MrMV_;
      mpFilter->mPrediction_.qVM_ = q.q_;
      mpFilter->mPrediction_.MrMV_ = vec.v_;
      for(unsigned int i=0;i<nMax_;i++){
        testState.template get<mtState::_aux>().fManager_.addFeature(feature);
        testState.template get<mtState::_aux>().fManager_.features_[i].foundInImage_ = true;
        testState.template get<mtState::_aux>().fManager_.features_[i].c_ = cv::Point2f((i*39829)%250,(i*49922)%250);
      }
      testState.template get<mtState::_aux>().fManager_.features_[1].foundInImage_ = false;
      testState.template get<mtState::_aux>().fManager_.removeFeature(0);
      predictionMeas_.setRandom(s);
      imgUpdateMeas_.setRandom(s);

//      std::cout << testState.template get<mtState::_dep>(15) << std::endl;
//      std::cout << testState.template getId<mtState::_dep>(15) << std::endl;
//      std::cout << testState.template get<mtState::_nor>(15).getVec().transpose() << std::endl;
//      std::cout << testState.template getId<mtState::_nor>(15) << std::endl; // TODO: investigate error for regular depth parametrization
//      std::cout << predictionMeas_.template get<mtPredictionMeas::_acc>().transpose() << std::endl;
//      std::cout << predictionMeas_.template get<mtPredictionMeas::_gyr>().transpose() << std::endl;
//      std::cout << testState.template get<mtState::_vel>().transpose() << std::endl;
//
//      double dt = 0.1;
//      const Eigen::Vector3d imuRor = (predictionMeas_.template get<mtPredictionMeas::_gyr>()-testState.template get<mtState::_gyb>());
//      const Eigen::Vector3d dOmega = dt*imuRor;
//      const Eigen::Vector3d camRor = q.q_.rotate(imuRor);
//      const Eigen::Vector3d camVel = q.q_.rotate(Eigen::Vector3d(imuRor.cross(vec.v_)-testState.template get<mtState::_vel>()));
//      std::cout << imuRor.transpose() << std::endl;
//      std::cout << dOmega.transpose() << std::endl;
//      std::cout << camRor.transpose() << std::endl;
//      std::cout << camVel.transpose() << std::endl;
//      std::cout << (rovio::gSM(testState.template get<mtState::_nor>(15).getVec())*camVel).transpose() << std::endl;
//      std::cout << ((rovio::M3D::Identity()-testState.template get<mtState::_nor>(15).getVec()*testState.template get<mtState::_nor>(15).getVec().transpose())*camRor).transpose() << std::endl;
//      const Eigen::Vector3d dm = dt*(rovio::gSM(testState.template get<mtState::_nor>(15).getVec())*camVel/testState.template get<mtState::_dep>(15)
//          + (rovio::M3D::Identity()-testState.template get<mtState::_nor>(15).getVec()*testState.template get<mtState::_nor>(15).getVec().transpose())*camRor);
//      std::cout << dm << std::endl;

      mpFilter->mPrediction_.testJacs(testState,predictionMeas_,1e-8,1e-6,0,0.1);
      std::get<0>(mpFilter->mUpdates_).testJacs(testState,imgUpdateMeas_,1e-9,1e-6,0,0.1);

      mpFilter->mPrediction_.qVM_ = q_temp.q_;
      mpFilter->mPrediction_.MrMV_ = vec_temp.v_;

      cameraOutputCF_.testJacInput(testState,testState,1e-8,1e-6,0,0.1);
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
      mpFilter->addUpdateMeas<0>(imgUpdateMeas_,img->header.stamp.toSec());
      double lastImuTime;
      if(mpFilter->predictionTimeline_.getLastTime(lastImuTime)){
        auto rit = std::get<0>(mpFilter->updateTimelineTuple_).measMap_.rbegin();
        while(rit != std::get<0>(mpFilter->updateTimelineTuple_).measMap_.rend() && rit->first > lastImuTime){
          ++rit;
        }
        if(rit != std::get<0>(mpFilter->updateTimelineTuple_).measMap_.rend()){
          updateAndPublish(rit->first);
        }
      }
    }
  }
  void updateAndPublish(const double& updateTime){
    if(isInitialized_){
      mpFilter->updateSafe(updateTime);
//      std::cout << "Filter calibration: " << std::endl;
//      std::cout << mpFilter->safe_.state_.template get<mtState::_acb>().transpose() << std::endl;
//      std::cout << mpFilter->safe_.state_.template get<mtState::_gyb>().transpose() << std::endl;
//      std::cout << mpFilter->safe_.state_.template get<mtState::_vep>().transpose() << std::endl;
//      std::cout << mpFilter->safe_.state_.template get<mtState::_vea>() << std::endl;
      if(!mpFilter->safe_.state_.template get<mtState::_aux>().img_.empty()){
        cv::imshow("Tracker", mpFilter->safe_.state_.template get<mtState::_aux>().img_);
        cv::waitKey(1);
      }
      if(pubPose_.getNumSubscribers() > 0){
        output_ = cameraOutputCF_.transformState(mpFilter->safe_.state_);
        outputCov_ = cameraOutputCF_.transformCovMat(mpFilter->safe_.state_,mpFilter->safe_.cov_);


        poseMsg_.header.seq = poseMsgSeq_++;
        poseMsg_.header.stamp = ros::Time(mpFilter->safe_.t_);

        poseMsg_.pose.position.x = output_.template get<mtOutput::_pos>()(0);
        poseMsg_.pose.position.y = output_.template get<mtOutput::_pos>()(1);
        poseMsg_.pose.position.z = output_.template get<mtOutput::_pos>()(2);
        poseMsg_.pose.orientation.w = output_.template get<mtOutput::_att>().w();
        poseMsg_.pose.orientation.x = output_.template get<mtOutput::_att>().x();
        poseMsg_.pose.orientation.y = output_.template get<mtOutput::_att>().y();
        poseMsg_.pose.orientation.z = output_.template get<mtOutput::_att>().z();
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
