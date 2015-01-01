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

#include <Eigen/Dense>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros-camera.h>
#include <unordered_set>

namespace rovio{

struct lk_landmark {
  std::vector<cv::Point2f> landmarks_;
  std::vector<unsigned int> idx_;
};

struct TrackedKeypointWithID {
  TrackedKeypointWithID(
      unsigned int _id,
      const Eigen::Matrix<double, 2, 1>& _location_previous_frame,
      const Eigen::Matrix<double, 2, 1>& _location_current_frame) :
    id(_id), location_previous_frame(_location_previous_frame),
    location_current_frame(_location_current_frame) { }
  unsigned int id;
  Eigen::Matrix<double, 2, 1> location_previous_frame;
  Eigen::Matrix<double, 2, 1> location_current_frame;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline bool in_image_with_border(const cv::Mat& im, const cv::Point2f& point,
                                 float border) {
  return point.x >= border && point.y >= border && point.x < im.cols - border
      && point.y < im.rows - border;
}

inline bool in_image(const cv::Mat& im, const cv::Point2f& point) {
  return point.x >= 0 && point.y >= 0 && point.x < im.cols && point.y < im.rows;
}
inline float FindShiTomasiScoreAtPoint(const cv::Mat& image, int nHalfBoxSize,
                                       const cv::Point2i& irCenter) {
  float dXX = 0;
  float dYY = 0;
  float dXY = 0;
  // Add 1 for the gradient.
  if (!in_image_with_border(image, irCenter, (float) nHalfBoxSize + 1)) {
    return -1.f;  // If not in image, return bad score.
  }

  cv::Point2i irStart = irCenter - cv::Point2i(nHalfBoxSize, nHalfBoxSize);
  cv::Point2i irEnd = irCenter + cv::Point2i(nHalfBoxSize, nHalfBoxSize);

  const uchar* ptr = image.data;
  auto& step = image.step;

  auto dataGetter = [ptr, step](const cv::Point2i& pt) -> uchar {
    return (ptr + (step.p[0]*pt.y))[pt.x];
  };

  cv::Point2i ir;
  for (ir.y = irStart.y; ir.y <= irEnd.y; ++ir.y) {
    for (ir.x = irStart.x; ir.x <= irEnd.x; ++ir.x) {
      float dx = dataGetter(ir + cv::Point2i(1, 0))
          - dataGetter(ir - cv::Point2i(1, 0));
      float dy = dataGetter(ir + cv::Point2i(0, 1))
          - dataGetter(ir - cv::Point2i(0, 1));
      dXX += dx * dx;
      dYY += dy * dy;
      dXY += dx * dy;
    }
  }

  int nPixels = (irEnd - irStart + cv::Point2i(1, 1)).x
      * (irEnd - irStart + cv::Point2i(1, 1)).y;
  dXX = dXX / (2.0f * nPixels);
  dYY = dYY / (2.0f * nPixels);
  dXY = dXY / (2.0f * nPixels);

  // Find and return smaller eigenvalue:
  return 0.5f
      * (dXX + dYY
          - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
}
template<typename Compare>
void distributeUniformly(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& current_keypoints, int imgwidth,
                         int imgheight, double radius, Compare compare) {

  cv::Mat _LUT = cv::Mat::zeros(2 * 16 - 1, 2 * 16 - 1, CV_32F);
  for (int x = 0; x < 2 * 16 - 1; ++x) {
    for (int y = 0; y < 2 * 16 - 1; ++y) {
      _LUT.at<float>(y, x) = std::max(1- double((radius / 2.0 - x) * (radius / 2.0 - x) + (radius / 2.0 - y) * (radius / 2.0 - y)) / double(radius / 2.0 * radius / 2.0), 0.0);
    }
  }

  // Sort.
  std::sort(keypoints.begin(), keypoints.end(), compare);
  std::cout << "Max score: " << keypoints.begin()->response << std::endl;
  std::cout << "Min score: " << keypoints.rbegin()->response << std::endl;

  std::vector<cv::KeyPoint> keypoints_new;
  keypoints_new.reserve(keypoints.size());

  // Store occupancy.
  cv::Mat occupancy;
  occupancy = cv::Mat::zeros((imgheight) / 2 + 32, (imgwidth) / 2 + 32, CV_8U);

  // Go through the sorted keypoints and reject too close ones.
  for (std::vector<cv::KeyPoint>::iterator it = keypoints.begin();
      it != keypoints.end(); ++it) {
    const int cy = (it->pt.y / 2 + 16);
    const int cx = (it->pt.x / 2 + 16);

    // Check if this is a high enough score.
    const double s0 = double(occupancy.at<uchar>(cy, cx));
    const double s1 = s0 * s0;
    if (short(it->response) < s1 * s1)
      continue;

    // Masks.
    const float nsc = sqrt(sqrt(it->response));
    for (int y = 0; y < 2 * 16 - 1; ++y) {
      __m128i mem1 = _mm_loadu_si128((__m128i *) &occupancy.at<uchar>(cy + y - 15, cx - 15));
      __m128i mem2 = _mm_loadu_si128((__m128i *) &occupancy.at<uchar>(cy + y - 15, cx + 1));
      __m128i mask1 = _mm_set_epi8(
          static_cast<uint8_t>(_LUT.at<float>(y, 15) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 14) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 13) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 12) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 11) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 10) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 9) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 8) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 7) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 6) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 5) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 4) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 3) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 2) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 1) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 0) * nsc));
      __m128i mask2 = _mm_set_epi8(
          static_cast<uint8_t>(_LUT.at<float>(y, 30) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 29) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 28) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 27) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 26) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 25) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 24) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 23) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 22) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 21) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 20) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 19) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 18) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 17) * nsc),
          static_cast<uint8_t>(_LUT.at<float>(y, 16) * nsc), 0);
      _mm_storeu_si128((__m128i *) &occupancy.at<uchar>(cy + y - 15, cx - 15),_mm_adds_epu8(mem1, mask1));
      _mm_storeu_si128((__m128i *) &occupancy.at<uchar>(cy + y - 15, cx + 1),_mm_adds_epu8(mem2, mask2));
    }
    keypoints_new.push_back(*it);
  }
  keypoints.swap(keypoints_new);
}

void distributeUniformlyMic(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& current_keypoints) {
  float maxResponse = -1.0;
  for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
    if(it->response > maxResponse) maxResponse = it->response;
  }

  const unsigned int nBuckets = 100; // TODO param
  std::unordered_set<cv::KeyPoint*> buckets[nBuckets];

  unsigned int newBucketID;
  for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
    if(it->response > 0.0){
      newBucketID = std::ceil(nBuckets*(it->response/maxResponse))-1;
      if(newBucketID>nBuckets-1) newBucketID = nBuckets-1;
      buckets[newBucketID].insert(&(*it));
    }
  }

  double distance;
  double maxDistance = 100; // TODO param
  double zeroDistancePenalty = nBuckets*1.0; // TODO param
  cv::KeyPoint* mpKeyPoint;
  for (auto it_current = current_keypoints.begin(); it_current != current_keypoints.end(); ++it_current) {
    for (unsigned int bucketID = 1;bucketID < nBuckets;bucketID++) {
      auto it = buckets[bucketID].begin();
      while(it != buckets[bucketID].end()) {
        mpKeyPoint = *it;
        it++;
        distance = std::sqrt(std::pow(it_current->x - mpKeyPoint->pt.x,2) + std::pow(it_current->y - mpKeyPoint->pt.y,2));
        if(distance<maxDistance){
          newBucketID = std::max((int)(bucketID - (maxDistance-distance)/maxDistance*zeroDistancePenalty),0);
          if(bucketID != newBucketID){
            buckets[newBucketID].insert(mpKeyPoint);
            buckets[bucketID].erase(mpKeyPoint);
          }
        }
      }
    }
  }

  std::vector<cv::KeyPoint> keypoints_new;
  keypoints_new.reserve(keypoints.size());
  cv::KeyPoint* mpKeyPoint2;
  for (int bucketID = nBuckets-1;bucketID >= 0;bucketID--) {
    while(!buckets[bucketID].empty()) {
      mpKeyPoint = *(buckets[bucketID].begin());
      buckets[bucketID].erase(mpKeyPoint);
      keypoints_new.push_back(*mpKeyPoint);
      for (unsigned int bucketID2 = 1;bucketID2 <= bucketID;bucketID2++) {
        auto it = buckets[bucketID2].begin();
        while(it != buckets[bucketID2].end()) {
          mpKeyPoint2 = *it;
          it++;
          distance = std::sqrt(std::pow(mpKeyPoint->pt.x - mpKeyPoint2->pt.x,2) + std::pow(mpKeyPoint->pt.y - mpKeyPoint2->pt.y,2));
          if(distance<maxDistance){
            newBucketID = std::max((int)(bucketID2 - (maxDistance-distance)/maxDistance*zeroDistancePenalty),0);
            if(bucketID2 != newBucketID){
              buckets[newBucketID].insert(mpKeyPoint2);
              buckets[bucketID2].erase(mpKeyPoint2);
            }
          }
        }
      }
    }
  }

  keypoints.swap(keypoints_new);
}

class TestNode{
 public:
  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg_;
  ros::Publisher pubPose_;
  unsigned int landmark_idx_;
  static constexpr int lk_max_level_ = 3;
  static constexpr double lk_min_eigen_threshold_ = 0.001;
  static constexpr double good_features_to_track_quality_level_ = 0.01;
  static constexpr double good_features_to_track_min_distance_ = 10.0;
  cv::Mat curr_image_, prev_image_, draw_image_;
  bool draw_matches_, need_init_, use_fast_corners_;
  double uniformity_radius_;
  int fast_threshold_;
  unsigned int min_feature_count_, max_feature_count_;
  cv::TermCriteria termcrit_;
  cv::Size sub_pix_win_size_, win_size_;
  std::shared_ptr<lk_landmark> landmarks_prev_image_, landmarks_curr_image_;
  std::shared_ptr<cv::FeatureDetector> feature_detector_fast_;
  std::vector<TrackedKeypointWithID> feature_and_index_vec_;
  std::shared_ptr<RosCamera> camera_;
  TestNode(ros::NodeHandle& nh): nh_(nh), termcrit_(cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20,0.03)),
      sub_pix_win_size_(cv::Size(10, 10)),win_size_(cv::Size(31, 31)){
    camera_.reset(new RosCamera("fpga21_cam0.yaml"));
    subImu_ = nh_.subscribe("imuMeas", 1000, &TestNode::imuCallback,this);
    subImg_ = nh_.subscribe("/cam0/image_raw", 1000, &TestNode::imgCallback,this);
    pubPose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("filterPose", 1000);
    landmark_idx_ = 0;
    draw_matches_ = true;
    use_fast_corners_ = true;
    min_feature_count_ = 150;
    max_feature_count_ = 20; // Maximal number of feature which is added at a time (not total)
    fast_threshold_ = 10;
    uniformity_radius_ = 50;
    need_init_ = false;
    landmarks_prev_image_.reset(new lk_landmark);
    landmarks_curr_image_.reset(new lk_landmark);
    feature_detector_fast_.reset(new cv::FastFeatureDetector(fast_threshold_, true));
  };
  void updateSafe(){
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
    curr_image_ = cv_ptr->image;

    // Timing
    static double last_timeStamp = 0.0;
    double current_timeStamp = img->header.stamp.toSec();

    //If we just started up
    if (prev_image_.empty())
      prev_image_ = curr_image_;

    // Drawing
    if (draw_matches_)
      cv_ptr->image.copyTo(draw_image_);

    // Check if initialization of new feature is required
    if (landmarks_prev_image_->landmarks_.size() < min_feature_count_)
      need_init_ = true;

    // Initialize new features (on prev image)
    if (need_init_) {
      ROS_INFO_STREAM("adding keypoints");

      // Get points from detector
      std::vector<cv::Point2f> points_temp;
      if (!use_fast_corners_)
        DetectGfttCorners(prev_image_, points_temp);
      else
        DetectFastCorners(prev_image_, points_temp, landmarks_prev_image_->landmarks_);

      // Assign IDs to points and add them to vectors
      for (size_t i = 0; i < points_temp.size(); ++i) {
        landmarks_prev_image_->idx_.push_back(landmark_idx_++);
      }
      landmarks_prev_image_->landmarks_.insert(landmarks_prev_image_->landmarks_.end(), points_temp.begin(),points_temp.end());

      need_init_ = false;
    }

    ROS_INFO_STREAM(" landmark size: " << landmarks_prev_image_->landmarks_.size() << " " << landmarks_prev_image_->idx_.size());

    // Find matches in current image
    if (!landmarks_prev_image_->landmarks_.empty()) {
      std::vector<uchar> status;
      std::vector<float> err;

      // Sparse Optical Flow
      cv::calcOpticalFlowPyrLK(prev_image_, curr_image_, landmarks_prev_image_->landmarks_, landmarks_curr_image_->landmarks_,
                               status, err, win_size_, lk_max_level_, termcrit_, 0, lk_min_eigen_threshold_);
      size_t i, k;

      // Feature matches and indices, used for plotting
      feature_and_index_vec_.clear();
      feature_and_index_vec_.reserve(landmarks_curr_image_->landmarks_.size());

      // Copy indices of previous to current
      landmarks_curr_image_->idx_ = landmarks_prev_image_->idx_;
//
//      feature_tracker::optical_flow_measurementPtr of_measurementmsg(
//          new feature_tracker::optical_flow_measurement);
//      FILE * pFile;
//      pFile = fopen("vi_data.txt", "a");
//      fprintf(pFile, "New Frame:\t%.3f\n", img->header.stamp.toSec());
      for (i = k = 0; i < landmarks_curr_image_->landmarks_.size(); ++i) {
        if (!status[i])
          continue;

//        feature_tracker::flow_vector oneFlow;
//
        // Just for plotting
        feature_and_index_vec_.emplace_back(
            landmarks_prev_image_->idx_[i],
            Eigen::Matrix<double, 2, 1>(landmarks_prev_image_->landmarks_[i].x,
                                        landmarks_prev_image_->landmarks_[i].y),
            Eigen::Matrix<double, 2, 1>(landmarks_curr_image_->landmarks_[i].x,
                                        landmarks_curr_image_->landmarks_[i].y));

        // Removes invalid landmarks
        landmarks_curr_image_->landmarks_[k] =
            landmarks_curr_image_->landmarks_[i];
        landmarks_curr_image_->idx_[k] = landmarks_curr_image_->idx_[i];
        ++k;

        Eigen::Vector3d dirvec_f2 = camera_->camToWorld(
            feature_and_index_vec_.back().location_current_frame.x(),
            feature_and_index_vec_.back().location_current_frame.y());
        Eigen::Vector3d dirvec_f1 = camera_->camToWorld(
            feature_and_index_vec_.back().location_previous_frame.x(),
            feature_and_index_vec_.back().location_previous_frame.y());

//        fprintf(pFile, "%.6f\t%.6f\t%.6f\t\t%.6f\t%.6f\t%.6f\t\n", dirvec_f1[0],
//                dirvec_f1[1], dirvec_f1[2], dirvec_f2[0], dirvec_f2[1],
//                dirvec_f2[2]);
//
//        oneFlow.p[0] = (dirvec_f2[0] + dirvec_f1[0])*0.5;
//        oneFlow.p[1] = (dirvec_f2[1] + dirvec_f1[1])*0.5;
//        oneFlow.p[2] = (dirvec_f2[2] + dirvec_f1[2])*0.5;
//
//        oneFlow.u[0] = (dirvec_f2[0] - dirvec_f1[0])/(current_timeStamp-last_timeStamp);
//        oneFlow.u[1] = (dirvec_f2[1] - dirvec_f1[1])/(current_timeStamp-last_timeStamp);
//        oneFlow.u[2] = (dirvec_f2[2] - dirvec_f1[2])/(current_timeStamp-last_timeStamp);
//
//        oneFlow.R = 0.5;  // ZMSSD distance.
//        of_measurementmsg->flow.push_back(oneFlow);
//

        // Drawing
        if (draw_matches_) {
          //TODO(omaris) Add flag to plot keypoint id or simply point
          if ((feature_and_index_vec_.back().id % 3) == 0) {
            cv::putText(
                draw_image_,
                std::to_string(feature_and_index_vec_.back().id),
                cv::Point(
                    feature_and_index_vec_.back().location_current_frame.x(),
                    feature_and_index_vec_.back().location_current_frame.y()),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));

            cv::line(
                draw_image_,
                cv::Point(
                    feature_and_index_vec_.back().location_current_frame.x(),
                    feature_and_index_vec_.back().location_current_frame.y()),
                cv::Point(
                    feature_and_index_vec_.back().location_previous_frame.x(),
                    feature_and_index_vec_.back().location_previous_frame.y()),
                cv::Scalar(255, 255, 0), 2);
            cv::circle(
                draw_image_,
                cv::Point(
                    feature_and_index_vec_.back().location_current_frame.x(),
                    feature_and_index_vec_.back().location_current_frame.y()),
                3, cv::Scalar(0, 0, 0), -1, 8);
          }
        }
      }
//      fclose(pFile);
//      static int seq = 0;
//      of_measurementmsg->header.stamp = img->header.stamp;
//      of_measurementmsg->header.frame_id = "/world";
//      of_measurementmsg->header.seq = seq++;
//      pub_flow_.publish(of_measurementmsg);
//
      // Remove the END
      landmarks_curr_image_->landmarks_.resize(k);
      landmarks_curr_image_->idx_.resize(k);
    } else {
      need_init_ = true;
    }
    if (draw_matches_) {
      cv::imshow("LK Tracker Window", draw_image_);
      cv::waitKey(3);
    }
    std::swap(landmarks_prev_image_, landmarks_curr_image_);
    cv::swap(prev_image_, curr_image_);
    last_timeStamp = current_timeStamp;
  }


  void DetectFastCorners(cv::Mat& img, std::vector<cv::Point2f>& detected_keypoints, std::vector<cv::Point2f>& current_keypoints) {
    std::vector<cv::KeyPoint> keypoints;
    double t1 = (double) cv::getTickCount();
    feature_detector_fast_->detect(img, keypoints);
    ROS_INFO_STREAM("Found using fast corners " << keypoints.size() << " max features " << max_feature_count_);

    // Sort by y for row corner LUT.
    std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool {return a.pt.y < b.pt.y;});
    double t2 = (double) cv::getTickCount();

    for (auto it = keypoints.begin(), end = keypoints.end(); it != end; ++it) {
      // Compute Shi-Tomasi score (returns negative if outside of image).
      it->response = FindShiTomasiScoreAtPoint(img, 3, it->pt);
    }
    double t3 = cv::getTickCount();
    distributeUniformlyMic(keypoints, current_keypoints);
//    distributeUniformly(keypoints, current_keypoints, img.cols, img.rows, uniformity_radius_, [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool {return a.response > b.response;});
    double t4 = cv::getTickCount();

    if (keypoints.size() > max_feature_count_) {
      keypoints.resize(max_feature_count_);  // Chop!
    }
    detected_keypoints.reserve(keypoints.size());

    for (auto it = keypoints.cbegin(), end = keypoints.cend(); it != end; ++it) {
      detected_keypoints.emplace_back(it->pt.x, it->pt.y);
    }
    ROS_INFO_STREAM("fast corners: " << (t2-t1)/cv::getTickFrequency() << " s, shi tomasi: " << (t3-t2)/cv::getTickFrequency()<< " s, distribute: " << (t4-t3)/cv::getTickFrequency() << " s");
  }

  void DetectGfttCorners(cv::Mat& img, std::vector<cv::Point2f>& detected_keypoints) {
    double t1 = (double) cv::getTickCount();
    // do something ...
    std::vector<unsigned int> landmark_idx_vec_temp;
    cv::goodFeaturesToTrack( prev_image_, detected_keypoints, max_feature_count_, good_features_to_track_quality_level_, good_features_to_track_min_distance_, cv::Mat());
    double t2 = (double) cv::getTickCount();
    cv::cornerSubPix(prev_image_, detected_keypoints, sub_pix_win_size_, cv::Size(-1, -1), termcrit_);
    double t3 = (double) cv::getTickCount();
    ROS_INFO_STREAM("Timings: gftt: " << (t2-t1)/cv::getTickFrequency() << "subpix refinement " << (t3-t2)/cv::getTickFrequency());
  }
};
}


#endif /* ROVIO_NODE_HPP_ */
