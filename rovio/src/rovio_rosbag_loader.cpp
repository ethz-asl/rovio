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

#include <memory>
#include <iostream>
#include <locale>
#include <string>
#include <Eigen/StdVector>
#include <chrono>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 8; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rovio_rosbag_loader");

  std::string rootdir = ament_index_cpp::get_package_share_directory("rovio");
  std::string filter_config = rootdir + "/cfg/rovio.info";

  node->declare_parameter("filter_config", filter_config);
  node->get_parameter("filter_config", filter_config);

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    std::string param_name = "camera" + std::to_string(camID) + "_config";
    node->declare_parameter(param_name, "");
    if (node->get_parameter(param_name, camera_config) && !camera_config.empty()) {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
    }
  }
  mpFilter->refreshProperties();

  // Node
  rovio::RovioNode<mtFilter> rovioNode(node, mpFilter);
  rovioNode.makeTest();

  double resetTrigger = 0.0;
  node->declare_parameter("record_odometry", rovioNode.forceOdometryPublishing_);
  node->get_parameter("record_odometry", rovioNode.forceOdometryPublishing_);
  node->declare_parameter("record_pose_with_covariance_stamped", rovioNode.forcePoseWithCovariancePublishing_);
  node->get_parameter("record_pose_with_covariance_stamped", rovioNode.forcePoseWithCovariancePublishing_);
  node->declare_parameter("record_transform", rovioNode.forceTransformPublishing_);
  node->get_parameter("record_transform", rovioNode.forceTransformPublishing_);
  node->declare_parameter("record_extrinsics", rovioNode.forceExtrinsicsPublishing_);
  node->get_parameter("record_extrinsics", rovioNode.forceExtrinsicsPublishing_);
  node->declare_parameter("record_imu_bias", rovioNode.forceImuBiasPublishing_);
  node->get_parameter("record_imu_bias", rovioNode.forceImuBiasPublishing_);
  node->declare_parameter("record_pcl", rovioNode.forcePclPublishing_);
  node->get_parameter("record_pcl", rovioNode.forcePclPublishing_);
  node->declare_parameter("record_markers", rovioNode.forceMarkersPublishing_);
  node->get_parameter("record_markers", rovioNode.forceMarkersPublishing_);
  node->declare_parameter("record_patch", rovioNode.forcePatchPublishing_);
  node->get_parameter("record_patch", rovioNode.forcePatchPublishing_);
  node->declare_parameter("reset_trigger", resetTrigger);
  node->get_parameter("reset_trigger", resetTrigger);

  std::cout << "Recording";
  if(rovioNode.forceOdometryPublishing_) std::cout << ", odometry";
  if(rovioNode.forceTransformPublishing_) std::cout << ", transform";
  if(rovioNode.forceExtrinsicsPublishing_) std::cout << ", extrinsics";
  if(rovioNode.forceImuBiasPublishing_) std::cout << ", imu biases";
  if(rovioNode.forcePclPublishing_) std::cout << ", point cloud";
  if(rovioNode.forceMarkersPublishing_) std::cout << ", markers";
  if(rovioNode.forcePatchPublishing_) std::cout << ", patch data";
  std::cout << std::endl;

  rosbag2_cpp::Reader bagIn;
  std::string rosbag_path = "dataset_ros2";
  node->declare_parameter("rosbag_path", rosbag_path);
  node->get_parameter("rosbag_path", rosbag_path);
  bagIn.open(rosbag_path);

  std::size_t found = rosbag_path.find_last_of("/");
  std::string base_path = rosbag_path.substr(0, found);
  std::string bag_dir_name = rosbag_path.substr(found + 1);
  if(base_path == rosbag_path){
    base_path = ".";
    bag_dir_name = rosbag_path;
  }

  // Modern C++ Time Formatting (Replaces Boost)
  std::stringstream stream;
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  stream << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S")
         << "_" << nMax_ << "_" << nLevels_ << "_" << patchSize_ << "_" << nCam_  << "_" << nPose_;

  std::string output_base_path = base_path + "/rovio/" + stream.str();
  node->declare_parameter("output_base_path", output_base_path);
  node->get_parameter("output_base_path", output_base_path);

  // ROS 2 bags are directories, so we use _bag
  std::string rosbag_out_dir = output_base_path + "_bag";
  std::string info_filename_out = output_base_path + ".info";
  std::cout << "Storing output to: " << rosbag_out_dir << std::endl;

  // Determine if we need to write at all
  bool forceAppendToBag = rovioNode.forceOdometryPublishing_ || rovioNode.forceTransformPublishing_ || rovioNode.forceExtrinsicsPublishing_ || rovioNode.forceImuBiasPublishing_ || rovioNode.forcePclPublishing_ || rovioNode.forceMarkersPublishing_ || rovioNode.forcePatchPublishing_;
  std::unique_ptr<rosbag2_cpp::Writer> bagOut;
  if(forceAppendToBag){
    bagOut = std::make_unique<rosbag2_cpp::Writer>();
    bagOut->open(rosbag_out_dir);
  }

  // Copy info
  std::ifstream  src(filter_config, std::ios::binary);
  std::ofstream  dst(info_filename_out,   std::ios::binary);
  dst << src.rdbuf();

  std::string imu_topic_name = "/imu0";
  node->declare_parameter("imu_topic_name", imu_topic_name);
  node->get_parameter("imu_topic_name", imu_topic_name);

  std::string cam0_topic_name = "/cam0/image_raw";
  node->declare_parameter("cam0_topic_name", cam0_topic_name);
  node->get_parameter("cam0_topic_name", cam0_topic_name);

  std::string cam1_topic_name = "/cam1/image_raw";
  node->declare_parameter("cam1_topic_name", cam1_topic_name);
  node->get_parameter("cam1_topic_name", cam1_topic_name);

  std::string odometry_topic_name = rovioNode.pubOdometry_->get_topic_name();
  std::string transform_topic_name = rovioNode.pubTransform_->get_topic_name();
  std::string extrinsics_topic_name[mtFilter::mtState::nCam_];
  for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
    extrinsics_topic_name[camID] = rovioNode.pubExtrinsics_[camID]->get_topic_name();
  }
  std::string imu_bias_topic_name = rovioNode.pubImuBias_->get_topic_name();
  std::string pcl_topic_name = rovioNode.pubPcl_->get_topic_name();
  std::string u_rays_topic_name = rovioNode.pubMarkers_->get_topic_name();
  std::string patch_topic_name = rovioNode.pubPatch_->get_topic_name();

  // Apply filter so the reader only returns these topics
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(imu_topic_name);
  filter.topics.push_back(cam0_topic_name);
  filter.topics.push_back(cam1_topic_name);
  bagIn.set_filter(filter);


  bool isTriggerInitialized = false;
  double lastTriggerTime = 0.0;

  // ROS 2 Serialization helpers
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<sensor_msgs::msg::Image> img_serialization;

  while(rclcpp::ok() && bagIn.has_next()){
    auto m = bagIn.read_next();
    rclcpp::SerializedMessage serialized_msg(*m->serialized_data);

    if (m->topic_name == imu_topic_name || ("/" + m->topic_name == imu_topic_name)) {
      auto imuMsg = std::make_shared<sensor_msgs::msg::Imu>();
      imu_serialization.deserialize_message(&serialized_msg, imuMsg.get());
      rovioNode.imuCallback(imuMsg);
    }
    if (m->topic_name == cam0_topic_name || ("/" + m->topic_name == cam0_topic_name)) {
      auto imgMsg = std::make_shared<sensor_msgs::msg::Image>();
      img_serialization.deserialize_message(&serialized_msg, imgMsg.get());
      rovioNode.imgCallback0(imgMsg);
    }
    if (m->topic_name == cam1_topic_name || ("/" + m->topic_name == cam1_topic_name)) {
      auto imgMsg = std::make_shared<sensor_msgs::msg::Image>();
      img_serialization.deserialize_message(&serialized_msg, imgMsg.get());
      rovioNode.imgCallback1(imgMsg);
    }
    rclcpp::spin_some(node);

    if(rovioNode.gotFirstMessages_){
      static double lastSafeTime = rovioNode.mpFilter_->safe_.t_;
      if(rovioNode.mpFilter_->safe_.t_ > lastSafeTime){
        if(forceAppendToBag) {
          rclcpp::Time stamp = node->now();
          // rosbag2 write order is (msg, topic, timestamp)
          if(rovioNode.forceOdometryPublishing_) bagOut->write(rovioNode.odometryMsg_, odometry_topic_name, stamp);
          if(rovioNode.forceTransformPublishing_) bagOut->write(rovioNode.transformMsg_, transform_topic_name, stamp);
          for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
            if(rovioNode.forceExtrinsicsPublishing_) bagOut->write(rovioNode.extrinsicsMsg_[camID], extrinsics_topic_name[camID], stamp);
          }
          if(rovioNode.forceImuBiasPublishing_) bagOut->write(rovioNode.imuBiasMsg_, imu_bias_topic_name, stamp);
          if(rovioNode.forcePclPublishing_) bagOut->write(rovioNode.pclMsg_, pcl_topic_name, stamp);
          if(rovioNode.forceMarkersPublishing_) bagOut->write(rovioNode.markerMsg_, u_rays_topic_name, stamp);
          if(rovioNode.forcePatchPublishing_) bagOut->write(rovioNode.patchMsg_, patch_topic_name, stamp);
        }
        lastSafeTime = rovioNode.mpFilter_->safe_.t_;
      }
      if(!isTriggerInitialized){
        lastTriggerTime = lastSafeTime;
        isTriggerInitialized = true;
      }
      if(resetTrigger>0.0 && lastSafeTime - lastTriggerTime > resetTrigger){
        rovioNode.requestReset();
        rovioNode.mpFilter_->init_.state_.WrWM() = rovioNode.mpFilter_->safe_.state_.WrWM();
        rovioNode.mpFilter_->init_.state_.qWM() = rovioNode.mpFilter_->safe_.state_.qWM();
        lastTriggerTime = lastSafeTime;
      }
    }
  }

  bagIn.close();
  if(forceAppendToBag) bagOut->close();
  rclcpp::shutdown();
  return 0;
}
