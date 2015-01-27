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

#ifndef COMMON_VISION_HPP_
#define COMMON_VISION_HPP_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros-camera.h>
#include <unordered_set>

using namespace Eigen;

namespace rovio{

template<int size>
struct Patch {
  static const int size_ = size;
  uint8_t data_[size_*size_] __attribute__ ((aligned (16)));
};

inline bool in_image_with_border(const cv::Mat& im, const cv::Point2f& point, float border) { // Feature_tracker code
  return point.x >= border && point.y >= border && point.x < im.cols - border
      && point.y < im.rows - border;
}

inline float FindShiTomasiScoreAtPoint(const cv::Mat& image, int nHalfBoxSize, const cv::Point2i& irCenter) {  // Feature_tracker code
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
      float dx = dataGetter(ir + cv::Point2i(1, 0)) - dataGetter(ir - cv::Point2i(1, 0));
      float dy = dataGetter(ir + cv::Point2i(0, 1)) - dataGetter(ir - cv::Point2i(0, 1));
      dXX += dx * dx;
      dYY += dy * dy;
      dXY += dx * dy;
    }
  }

  int nPixels = (irEnd - irStart + cv::Point2i(1, 1)).x * (irEnd - irStart + cv::Point2i(1, 1)).y;
  dXX = dXX / (2.0f * nPixels);
  dYY = dYY / (2.0f * nPixels);
  dXY = dXY / (2.0f * nPixels);

  // Find and return smaller eigenvalue: // TODO: this could be adapted to multiple levels, maybe not smaller eigenvalue
  return 0.5f * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
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
  double maxDistance = 20; // TODO param
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

void DetectFastCorners(cv::Mat& img, std::vector<cv::Point2f>& detected_keypoints, std::vector<cv::Point2f>& current_keypoints, unsigned int maxN) {
  std::vector<cv::KeyPoint> keypoints;
  double t1 = (double) cv::getTickCount();
  cv::FastFeatureDetector feature_detector_fast(10, true); // TODO param
  feature_detector_fast.detect(img, keypoints);
  ROS_INFO_STREAM("Found using fast corners " << keypoints.size());

  // Sort by y for row corner LUT.
  std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool {return a.pt.y < b.pt.y;});
  double t2 = (double) cv::getTickCount();

  for (auto it = keypoints.begin(), end = keypoints.end(); it != end; ++it) {
    // Compute Shi-Tomasi score (returns negative if outside of image).
    it->response = FindShiTomasiScoreAtPoint(img, 3, it->pt);
  }
  double t3 = cv::getTickCount();
  distributeUniformlyMic(keypoints, current_keypoints);
//  distributeUniformly(keypoints, current_keypoints, img.cols, img.rows, 50, [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool {return a.response > b.response;});
  double t4 = cv::getTickCount();

  if (keypoints.size() > maxN) {
    keypoints.resize(maxN);  // Chop!
  }
  detected_keypoints.reserve(keypoints.size());

  for (auto it = keypoints.cbegin(), end = keypoints.cend(); it != end; ++it) {
    detected_keypoints.emplace_back(it->pt.x, it->pt.y);
  }
  ROS_INFO_STREAM("fast corners: " << (t2-t1)/cv::getTickFrequency() << " s, shi tomasi: " <<
                  (t3-t2)/cv::getTickFrequency()<< " s, distribute: " << (t4-t3)/cv::getTickFrequency() << " s");
}



bool align2D( // SVO code
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    Vector2d& cur_px_estimate)
{
  const int halfpatch_size_ = 4;
  const int patch_size_ = 8;
  const int patch_area_ = 64;
  bool converged=false;

  // compute derivative of template and prepare inverse compositional
  float __attribute__((__aligned__(16))) ref_patch_dx[patch_area_];
  float __attribute__((__aligned__(16))) ref_patch_dy[patch_area_];
  Matrix3f H; H.setZero();

  // compute gradient and hessian
  const int ref_step = patch_size_+2;
  float* it_dx = ref_patch_dx;
  float* it_dy = ref_patch_dy;
  for(int y=0; y<patch_size_; ++y)
  {
    uint8_t* it = ref_patch_with_border + (y+1)*ref_step + 1;
    for(int x=0; x<patch_size_; ++x, ++it, ++it_dx, ++it_dy)
    {
      Vector3f J;
      J[0] = 0.5 * (it[1] - it[-1]);
      J[1] = 0.5 * (it[ref_step] - it[-ref_step]);
      J[2] = 1;
      *it_dx = J[0];
      *it_dy = J[1];
      H += J*J.transpose();
    }
  }
  Matrix3f Hinv = H.inverse();
  float mean_diff = 0;

  // Compute pixel location in new image:
  float u = cur_px_estimate.x();
  float v = cur_px_estimate.y();

  // termination condition
  const float min_update_squared = 0.03*0.03;
  const int cur_step = cur_img.step.p[0];
  Vector3f update; update.setZero();
  for(int iter = 0; iter<n_iter; ++iter)
  {
    int u_r = floor(u);
    int v_r = floor(v);
    if(u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols-halfpatch_size_ || v_r >= cur_img.rows-halfpatch_size_)
      break;

    if(isnan(u) || isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
      return false;

    // compute interpolation weights
    float subpix_x = u-u_r;
    float subpix_y = v-v_r;
    float wTL = (1.0-subpix_x)*(1.0-subpix_y);
    float wTR = subpix_x * (1.0-subpix_y);
    float wBL = (1.0-subpix_x)*subpix_y;
    float wBR = subpix_x * subpix_y;

    // loop through search_patch, interpolate
    uint8_t* it_ref = ref_patch;
    float* it_ref_dx = ref_patch_dx;
    float* it_ref_dy = ref_patch_dy;
    Vector3f Jres; Jres.setZero();
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* it = (uint8_t*) cur_img.data + (v_r+y-halfpatch_size_)*cur_step + u_r-halfpatch_size_;
      for(int x=0; x<patch_size_; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy)
      {
        float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
        float res = search_pixel - *it_ref + mean_diff;
        Jres[0] -= res*(*it_ref_dx);
        Jres[1] -= res*(*it_ref_dy);
        Jres[2] -= res;
      }
    }

    update = Hinv * Jres;
    if(fabs(update[0])>1.0) update[0] = update[0]/fabs(update[0]);
    if(fabs(update[1])>1.0) update[1] = update[1]/fabs(update[1]);
    u += update[0];
    v += update[1];
    mean_diff += update[2];

    if(update[0]*update[0]+update[1]*update[1] < min_update_squared)
    {
      converged=true;
      break;
    }
  }

  cur_px_estimate << u, v;
  return converged;
}

void createPatchFromPatchWithBorder(
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch) // SVO code
{
  const int patch_size_ = 8;
  uint8_t* ref_patch_ptr = ref_patch;
  for(int y=1; y<patch_size_+1; ++y, ref_patch_ptr += patch_size_)
  {
    uint8_t* ref_patch_border_ptr = ref_patch_with_border + y*(patch_size_+2) + 1;
    for(int x=0; x<patch_size_; ++x)
      ref_patch_ptr[x] = ref_patch_border_ptr[x];
  }
}


bool getPatchInterpolated(
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int halfpatch_size,
    uint8_t* patch)
{
  const int patch_size = halfpatch_size*2;
  const int cur_step = img_ref.step.p[0];

  const float u = px_ref.x();
  const float v = px_ref.y();
  const int u_r = floor(u);
  const int v_r = floor(v);
  if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= img_ref.cols-halfpatch_size || v_r >= img_ref.rows-halfpatch_size)
    return false;

  // compute interpolation weights
  const float subpix_x = u-u_r;
  const float subpix_y = v-v_r;
  const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
  const float wTR = subpix_x * (1.0-subpix_y);
  const float wBL = (1.0-subpix_x)*subpix_y;
  const float wBR = subpix_x * subpix_y;

  // Extract Patch
  uint8_t* patch_ptr = patch;
  for(int y=0; y<patch_size; ++y){
    uint8_t* it = (uint8_t*) img_ref.data + (v_r+y-halfpatch_size)*cur_step + u_r-halfpatch_size;
    for(int x=0; x<patch_size; ++x, ++it, ++patch_ptr)
    {
      *patch_ptr = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
    }
  }
  return true;
}

}


#endif /* COMMON_VISION_HPP_ */
