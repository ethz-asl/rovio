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
#include <rovio/RovioOutput.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include "CameraOutputCF.hpp"
#include "YprOutputCF.hpp"


class TestFilter{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg_;
  ros::Publisher pubPose_;
  ros::Publisher pubRovioOutput_;
  ros::Publisher pubOdometry_;
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
  nav_msgs::Odometry odometryMsg_;
  rovio::RovioOutput rovioOutputMsg_;
  int poseMsgSeq_;
  typedef rovio::StandardOutput mtOutput;
  mtOutput output_;
  mtOutput::mtCovMat outputCov_;
  rovio::CameraOutputCF<typename mtFilter::mtPrediction> cameraOutputCF_;

  typedef rovio::AttitudeOutput mtAttitudeOutput;
  mtAttitudeOutput attitudeOutput_;
  mtAttitudeOutput::mtCovMat attitudeOutputCov_;
  typedef rovio::YprOutput mtYprOutput;
  mtYprOutput yprOutput_;
  mtYprOutput::mtCovMat yprOutputCov_;
  rovio::AttitudeToYprCF attitudeToYprCF_;

  TestFilter(ros::NodeHandle& nh): nh_(nh), mpFilter(new rovio::Filter<mtState>()), cameraOutputCF_(mpFilter->mPrediction_){
    #ifndef NDEBUG
      ROS_WARN("====================== Debug Mode ======================");
    #endif
    subImu_ = nh_.subscribe("/imu0", 1000, &TestFilter::imuCallback,this);
    subImg_ = nh_.subscribe("/cam0/image_raw", 1000, &TestFilter::imgCallback,this);
    pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/rovio/pose", 1);
    pubRovioOutput_ = nh_.advertise<rovio::RovioOutput>("/rovio/output", 1);
    pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("/rovio/odometry", 1);
    std::string rootdir = ros::package::getPath("rovio");
    mpFilter->readFromInfo(rootdir + "/cfg/rovio.info");

//    // p21012_equidist_190215
//    Eigen::Matrix3d R_VM;
//    R_VM << 0.9999327192366118, -0.0044421301653885, 0.010715618492127002,
//            0.0043735142886046205, 0.9999698383876616, 0.0064183087897756765,
//            -0.01074380625488192, -0.006371012050474087, 0.9999219873733199;
//    Eigen::Vector3d VrVM;
//    VrVM << -0.03386081589435305, 0.005807520979411451, 0.0005138746353526602;
//    // p21012_radtan_190215 (false)
//    Eigen::Matrix3d R_VM;
//    R_VM << 0.9999327192366118, -0.0044421301653885, 0.010715618492127002,
//            0.0043735142886046205, 0.9999698383876616, 0.0064183087897756765,
//            -0.01074380625488192, -0.006371012050474087, 0.9999219873733199;
//    Eigen::Vector3d VrVM;
//    VrVM << -0.03386081589435305, 0.005807520979411451, 0.0005138746353526602;
//    mpFilter->mPrediction_.setExtrinsics(R_VM,VrVM);

    poseMsg_.header.frame_id = "/world";
    rovioOutputMsg_.header.frame_id = "/world";
    rovioOutputMsg_.points.header.frame_id = "/camera";
    odometryMsg_.header.frame_id = "/world";
    odometryMsg_.child_frame_id = "/camera";
    poseMsgSeq_ = 1;
    cv::namedWindow("Tracker");
    isInitialized_ = false;

    bool makeTest = true;
    if(makeTest){
      mtState testState;
      unsigned int s = 2;
      testState.setRandom(s); // TODO: debug with   doVECalibration = false and depthType = 0
      rovio::MultilevelPatchFeature<nLevels_,patchSize_> feature;
      feature.increaseStatistics(0.0);
      feature.currentStatistics_.status_ = rovio::TrackingStatistics::FOUND;
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
      Eigen::Vector2d vec2;
      LWF::NormalVectorElement m_meas;
      for(unsigned int i=0;i<nMax_;i++){
        int ind = testState.template get<mtState::_aux>().fManager_.addFeature(feature);
        testState.template get<mtState::_aux>().fManager_.features_[ind].c_ = cv::Point2f((i*39829)%250,(i*49922)%250);
        std::get<0>(mpFilter->mUpdates_).mpCamera_->pixelToBearing(testState.template get<mtState::_aux>().fManager_.features_[ind].c_,m_meas);
        m_meas.boxMinus(testState.template get<mtState::_nor>(ind),vec2);
        testState.difVecLin_.template block<2,1>(mtState::template getId<mtState::_nor>(ind),0) = vec2;
      }
      testState.template get<mtState::_aux>().fManager_.features_[1].currentStatistics_.status_ = rovio::TrackingStatistics::NOTFOUND;
      testState.template get<mtState::_aux>().fManager_.removeFeature(0);
      predictionMeas_.setRandom(s);
      imgUpdateMeas_.setRandom(s);

//      std::cout << testState.template get<mtState::_dep>(15) << std::endl;
//      std::cout << testState.template getId<mtState::_dep>(15) << std::endl;
//      std::cout << testState.template get<mtState::_nor>(15).getVec().transpose() << std::endl;
//      std::cout << testState.template getId<mtState::_nor>(15) << std::endl;
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

      mpFilter->mPrediction_.testJacs(testState,predictionMeas_,1e-8,1e-6,0.1);
      std::get<0>(mpFilter->mUpdates_).testJacs(testState,imgUpdateMeas_,1e-9,1e-6,0.1);

      mpFilter->mPrediction_.qVM_ = q_temp.q_;
      mpFilter->mPrediction_.MrMV_ = vec_temp.v_;

      cameraOutputCF_.testJacInput(testState,testState,1e-8,1e-6,0.1);

      attitudeToYprCF_.testJacInput(1e-8,1e-6,s,0.1);
    }
  }
  ~TestFilter(){
    delete mpFilter;
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
      imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_ = img->header.stamp.toSec();
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
      if(pubPose_.getNumSubscribers() > 0 || pubRovioOutput_.getNumSubscribers() > 0){
        mtState& state = mpFilter->safe_.state_;
        mtState::mtCovMat& cov = mpFilter->safe_.cov_;
        output_ = cameraOutputCF_.transformState(state);
        outputCov_ = cameraOutputCF_.transformCovMat(state,cov);


        poseMsg_.header.seq = poseMsgSeq_;
        poseMsg_.header.stamp = ros::Time(mpFilter->safe_.t_);

        poseMsg_.pose.position.x = output_.template get<mtOutput::_pos>()(0);
        poseMsg_.pose.position.y = output_.template get<mtOutput::_pos>()(1);
        poseMsg_.pose.position.z = output_.template get<mtOutput::_pos>()(2);
        poseMsg_.pose.orientation.w = output_.template get<mtOutput::_att>().w();
        poseMsg_.pose.orientation.x = output_.template get<mtOutput::_att>().x();
        poseMsg_.pose.orientation.y = output_.template get<mtOutput::_att>().y();
        poseMsg_.pose.orientation.z = output_.template get<mtOutput::_att>().z();
        pubPose_.publish(poseMsg_);


        rovioOutputMsg_.header.seq = poseMsgSeq_;
        rovioOutputMsg_.header.stamp = ros::Time(mpFilter->safe_.t_);
        odometryMsg_.header.seq = poseMsgSeq_;
        odometryMsg_.header.stamp = ros::Time(mpFilter->safe_.t_);

        // Odometry
        odometryMsg_.pose.pose.position.x = output_.template get<mtOutput::_pos>()(0);
        odometryMsg_.pose.pose.position.y = output_.template get<mtOutput::_pos>()(1);
        odometryMsg_.pose.pose.position.z = output_.template get<mtOutput::_pos>()(2);
        odometryMsg_.pose.pose.orientation.w = output_.template get<mtOutput::_att>().w();
        odometryMsg_.pose.pose.orientation.x = output_.template get<mtOutput::_att>().x();
        odometryMsg_.pose.pose.orientation.y = output_.template get<mtOutput::_att>().y();
        odometryMsg_.pose.pose.orientation.z = output_.template get<mtOutput::_att>().z();
        for(unsigned int i=0;i<6;i++){
          unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>()+i;
          if(i>=3) ind1 = mtOutput::template getId<mtOutput::_att>()+i-3;
          for(unsigned int j=0;j<6;j++){
            unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>()+j;
            if(j>=3) ind2 = mtOutput::template getId<mtOutput::_att>()+j-3;
            odometryMsg_.pose.covariance[j+6*i] = outputCov_(ind1,ind2);
          }
        }
        odometryMsg_.twist.twist.linear.x = output_.template get<mtOutput::_vel>()(0);
        odometryMsg_.twist.twist.linear.y = output_.template get<mtOutput::_vel>()(1);
        odometryMsg_.twist.twist.linear.z = output_.template get<mtOutput::_vel>()(2);
        odometryMsg_.twist.twist.angular.x = output_.template get<mtOutput::_ror>()(0);
        odometryMsg_.twist.twist.angular.y = output_.template get<mtOutput::_ror>()(1);
        odometryMsg_.twist.twist.angular.z = output_.template get<mtOutput::_ror>()(2);
        for(unsigned int i=0;i<6;i++){
          unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>()+i;
          if(i>=3) ind1 = mtOutput::template getId<mtOutput::_ror>()+i-3;
          for(unsigned int j=0;j<6;j++){
            unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>()+j;
            if(j>=3) ind2 = mtOutput::template getId<mtOutput::_ror>()+j-3;
            odometryMsg_.twist.covariance[j+6*i] = outputCov_(ind1,ind2);
          }
        }
        rovioOutputMsg_.odometry = odometryMsg_;
        attitudeOutput_.template get<mtAttitudeOutput::_att>() = output_.template get<mtOutput::_att>();
        attitudeOutputCov_ = outputCov_.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtOutput::template getId<mtOutput::_att>());
        yprOutput_ = attitudeToYprCF_.transformState(attitudeOutput_);
        yprOutputCov_ = attitudeToYprCF_.transformCovMat(attitudeOutput_,attitudeOutputCov_);
        rovioOutputMsg_.ypr_odometry.x = yprOutput_.template get<mtYprOutput::_ypr>()(0);
        rovioOutputMsg_.ypr_odometry.y = yprOutput_.template get<mtYprOutput::_ypr>()(1);
        rovioOutputMsg_.ypr_odometry.z = yprOutput_.template get<mtYprOutput::_ypr>()(2);
        rovioOutputMsg_.ypr_odometry_sigma.x = yprOutput_.template get<mtYprOutput::_ypr>()(0)+3*yprOutputCov_(0,0);
        rovioOutputMsg_.ypr_odometry_sigma.y = yprOutput_.template get<mtYprOutput::_ypr>()(1)+3*yprOutputCov_(1,1);
        rovioOutputMsg_.ypr_odometry_sigma.z = yprOutput_.template get<mtYprOutput::_ypr>()(2)+3*yprOutputCov_(2,2);

        // IMU biases
        rovioOutputMsg_.acc_bias.x = state.template get<mtState::_acb>()(0);
        rovioOutputMsg_.acc_bias.y = state.template get<mtState::_acb>()(1);
        rovioOutputMsg_.acc_bias.z = state.template get<mtState::_acb>()(2);
        rovioOutputMsg_.acc_bias_sigma.x = cov(mtState::template getId<mtState::_acb>()+0,mtState::template getId<mtState::_acb>()+0);
        rovioOutputMsg_.acc_bias_sigma.y = cov(mtState::template getId<mtState::_acb>()+1,mtState::template getId<mtState::_acb>()+1);
        rovioOutputMsg_.acc_bias_sigma.z = cov(mtState::template getId<mtState::_acb>()+2,mtState::template getId<mtState::_acb>()+2);
        rovioOutputMsg_.gyr_bias.x = state.template get<mtState::_gyb>()(0);
        rovioOutputMsg_.gyr_bias.y = state.template get<mtState::_gyb>()(1);
        rovioOutputMsg_.gyr_bias.z = state.template get<mtState::_gyb>()(2);
        rovioOutputMsg_.gyr_bias_sigma.x = cov(mtState::template getId<mtState::_gyb>()+0,mtState::template getId<mtState::_gyb>()+0);
        rovioOutputMsg_.gyr_bias_sigma.y = cov(mtState::template getId<mtState::_gyb>()+1,mtState::template getId<mtState::_gyb>()+1);
        rovioOutputMsg_.gyr_bias_sigma.z = cov(mtState::template getId<mtState::_gyb>()+2,mtState::template getId<mtState::_gyb>()+2);

        // Extrinsics
        rovioOutputMsg_.extrinsics.pose.position.x = state.template get<mtState::_vep>()(0);
        rovioOutputMsg_.extrinsics.pose.position.y = state.template get<mtState::_vep>()(1);
        rovioOutputMsg_.extrinsics.pose.position.z = state.template get<mtState::_vep>()(2);
        rovioOutputMsg_.extrinsics.pose.orientation.w = state.template get<mtState::_vea>().w();
        rovioOutputMsg_.extrinsics.pose.orientation.x = state.template get<mtState::_vea>().x();
        rovioOutputMsg_.extrinsics.pose.orientation.y = state.template get<mtState::_vea>().y();
        rovioOutputMsg_.extrinsics.pose.orientation.z = state.template get<mtState::_vea>().z();
        for(unsigned int i=0;i<6;i++){
          unsigned int ind1 = mtState::template getId<mtState::_vep>()+i;
          if(i>=3) ind1 = mtState::template getId<mtState::_vea>()+i-3;
          for(unsigned int j=0;j<6;j++){
            unsigned int ind2 = mtState::template getId<mtState::_vep>()+j;
            if(j>=3) ind2 = mtState::template getId<mtState::_vea>()+j-3;
            rovioOutputMsg_.extrinsics.covariance[j+6*i] = cov(ind1,ind2);
          }
        }
        attitudeOutput_.template get<mtAttitudeOutput::_att>() = state.template get<mtState::_vea>();
        attitudeOutputCov_ = cov.template block<3,3>(mtState::template getId<mtState::_vea>(),mtState::template getId<mtState::_vea>());
        yprOutput_ = attitudeToYprCF_.transformState(attitudeOutput_);
        yprOutputCov_ = attitudeToYprCF_.transformCovMat(attitudeOutput_,attitudeOutputCov_);
        rovioOutputMsg_.ypr_extrinsics.x = yprOutput_.template get<mtYprOutput::_ypr>()(0);
        rovioOutputMsg_.ypr_extrinsics.y = yprOutput_.template get<mtYprOutput::_ypr>()(1);
        rovioOutputMsg_.ypr_extrinsics.z = yprOutput_.template get<mtYprOutput::_ypr>()(2);
        rovioOutputMsg_.ypr_extrinsics_sigma.x = yprOutputCov_(0,0);
        rovioOutputMsg_.ypr_extrinsics_sigma.y = yprOutputCov_(1,1);
        rovioOutputMsg_.ypr_extrinsics_sigma.z = yprOutputCov_(2,2);

        //Point cloud
        rovio::FeatureManager<mtState::nLevels_,mtState::patchSize_,mtState::nMax_>& fManager = state.template get<mtState::_aux>().fManager_;
        rovioOutputMsg_.points.header.seq = poseMsgSeq_;
        rovioOutputMsg_.points.header.stamp = ros::Time(mpFilter->safe_.t_);
        rovioOutputMsg_.points.height = 1;
        rovioOutputMsg_.points.width = fManager.validSet_.size();
        for(auto it_f = fManager.validSet_.begin();it_f != fManager.validSet_.end(); ++it_f){

        }

        pubRovioOutput_.publish(rovioOutputMsg_);
        pubOdometry_.publish(odometryMsg_);
        poseMsgSeq_++;
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
