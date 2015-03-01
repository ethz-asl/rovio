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
#include <Eigen/Cholesky>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_set>
#include <Eigen/Eigenvalues>

using namespace Eigen;

namespace rovio{

class TrackingStatistics{
 public:
  TrackingStatistics(){
    inFrame_ = false;
    status_ = NOTFOUND;
  };
  ~TrackingStatistics(){};
  bool inFrame_;
  enum Status{
    TRACKED,
    OUTLIER,
    FOUND,
    INIT,
    NOTFOUND
  } status_;
};

class DrawPoint{
 public:
  cv::Point2f c_;
  double sigma1_;
  double sigma2_;
  double sigmaAngle_;
  bool hasSigma_;
  double factor_;
  Eigen::EigenSolver<Eigen::Matrix2d> es_;
  DrawPoint(){
    c_ = cv::Point2f(0,0);
    sigma1_ = 1.0;
    sigma2_ = 1.0;
    hasSigma_ = false;
    sigmaAngle_ = 0.0;
    factor_ = 2.0;
  }
  void draw(cv::Mat& drawImg,const cv::Scalar& color){
    cv::Size size(2,2);
    cv::ellipse(drawImg,c_,size,0,0,360,color,-1,8,0);
    if(hasSigma_){
      cv::ellipse(drawImg,c_,cv::Size(std::max(static_cast<int>(factor_*sigma1_+0.5),1),std::max(static_cast<int>(factor_*sigma2_+0.5),1)),sigmaAngle_,0,360,color,1,8,0);
    }
  }
  void drawLine(cv::Mat& drawImg,const DrawPoint& p,const cv::Scalar& color, int thickness = 2){
    cv::line(drawImg,c_,p.c_,color, thickness);
  }
  void drawText(cv::Mat& drawImg,const std::string& s,const cv::Scalar& color){
    cv::putText(drawImg,s,c_,cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
  }
  void setSigmaFromCov(const Eigen::Matrix2d& cov){
    es_.compute(cov);
    sigmaAngle_ = std::atan2(es_.eigenvectors()(1,0).real(),es_.eigenvectors()(0,0).real())*180/M_PI;
    sigma1_ =  sqrt(es_.eigenvalues()(0).real());
    sigma2_  = sqrt(es_.eigenvalues()(1).real());
    hasSigma_ = true;
  }
};

template<int n_levels>
class ImagePyramid{
 public:
  ImagePyramid(){};
  ~ImagePyramid(){};
  cv::Mat imgs_[n_levels];
  void computeFromImage(const cv::Mat& img){
    img.copyTo(imgs_[0]);
    for(int i=1; i<n_levels; ++i){
      cv::pyrDown(imgs_[i-1],imgs_[i],cv::Size(imgs_[i-1].cols/2, imgs_[i-1].rows/2));
    }
  }
  ImagePyramid<n_levels>& operator=(const ImagePyramid<n_levels> &rhs) {
    for(unsigned int i=0;i<n_levels;i++){
      rhs.imgs_[i].copyTo(imgs_[i]);
    }
    return *this;
  }
};

template<int size>
class PatchNew {
 public:
  static constexpr int size_ = size;
  uint8_t patch_[size_*size_] __attribute__ ((aligned (16)));
  uint8_t patchWithBorder_[(size_+2)*(size_+2)] __attribute__ ((aligned (16)));
  float dx_[size_*size_] __attribute__ ((aligned (16)));
  float dy_[size_*size_] __attribute__ ((aligned (16)));
  Matrix3f H_;
  float s_;
  bool validGradientParameters;
  bool hasValidPatch_; // Patch gets invalidates if extractPatchFromImage is not succesfull
  PatchNew(){
    static_assert(size_%2==0,"Patch size must be a multiple of 2");
    validGradientParameters = false;
    s_ = 0.0;
    hasValidPatch_ = false;
  }
  ~PatchNew(){}
  void computeGradientParameters(){
    if(validGradientParameters)
      return;
    H_.setZero();
    const int refStep = size_+2;
    float* it_dx = dx_;
    float* it_dy = dy_;
    uint8_t* it;
    Vector3f J;
    for(int y=0; y<size_; ++y){
      it = patchWithBorder_ + (y+1)*refStep + 1;
      for(int x=0; x<size_; ++x, ++it, ++it_dx, ++it_dy){
        J[0] = 0.5 * (it[1] - it[-1]);
        J[1] = 0.5 * (it[refStep] - it[-refStep]);
        J[2] = 1;
        *it_dx = J[0];
        *it_dy = J[1];
        H_ += J*J.transpose();
      }
    }
    const float dXX = H_(0,0)/(size_*size_);
    const float dYY = H_(1,1)/(size_*size_);
    const float dXY = H_(0,1)/(size_*size_);
    // Find and return smaller eigenvalue:
    s_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    validGradientParameters = true;
  }
  bool extractPatchFromImage(const cv::Mat& img,const cv::Point2f c){
    const int halfpatch_size = size_/2+1;
    const int refStep = img.step.p[0];
    const float u = c.x;
    const float v = c.y;
    const int u_r = floor(u);
    const int v_r = floor(v);
    if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= img.cols-halfpatch_size || v_r >= img.rows-halfpatch_size){
      hasValidPatch_ = false;
      return false;
    }

    // compute interpolation weights
    const float subpix_x = u-u_r;
    const float subpix_y = v-v_r;
    const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
    const float wTR = subpix_x * (1.0-subpix_y);
    const float wBL = (1.0-subpix_x)*subpix_y;
    const float wBR = subpix_x * subpix_y;

    // Extract Patch
    uint8_t* patch_ptr = patchWithBorder_;
    uint8_t* img_ptr;
    for(int y=0; y<(size_+2); ++y){
      img_ptr = (uint8_t*) img.data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
      for(int x=0; x<(size_+2); ++x, ++img_ptr, ++patch_ptr)
      {
        *patch_ptr = wTL*img_ptr[0] + wTR*img_ptr[1] + wBL*img_ptr[refStep] + wBR*img_ptr[refStep+1];
      }
    }

    extractPatchFromPatchWithBorder();
    validGradientParameters = false;
    hasValidPatch_ = true;
    return true;
  }
  void extractPatchFromPatchWithBorder(){
    uint8_t* it_patch = patch_;
    uint8_t* it_patchWithBorder;
    for(int y=1; y<size_+1; ++y, it_patch += size_){
      it_patchWithBorder = patchWithBorder_ + y*(size_+2) + 1;
      for(int x=0; x<size_; ++x)
        it_patch[x] = it_patchWithBorder[x];
    }
  }
  float getScore(){
    if(!validGradientParameters) computeGradientParameters();
    return s_;
  }
  Eigen::Matrix3f getHessian(){
    if(!validGradientParameters) computeGradientParameters();
    return H_;
  }
};

template<int n_levels,int patch_size>
class MultilevelPatchFeature{
 public:
  static const int nLevels_ = n_levels;
  PatchNew<patch_size> patches_[nLevels_];
  cv::Point2f c_;
  static constexpr float warpDistance = static_cast<float>(patch_size);
  cv::Point2f corners_[2];
  int idx_;
  Matrix3f H_;
  float s_;
  bool hasValidPatches_;
  TrackingStatistics currentStatistics_;
  std::map<double,TrackingStatistics> trackingStatistics_;
  std::map<TrackingStatistics::Status,int> cumulativeStatistics_; // Should be 0 initialized
  int totCount_;

  DrawPoint log_previous_;
  DrawPoint log_prediction_;
  DrawPoint log_predictionC0_;
  DrawPoint log_predictionC1_;
  DrawPoint log_predictionC2_;
  DrawPoint log_predictionC3_;
  DrawPoint log_meas_;
  DrawPoint log_current_;

  Eigen::MatrixXf A_;
  Eigen::MatrixXf b_;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXf> mColPivHouseholderQR_;

  void increaseStatistics(const double& t){
    if(!trackingStatistics_.empty() && t<trackingStatistics_.rbegin()->first) std::cout << "Warning: adding statistics NOT at end" << std::endl;
    cumulativeStatistics_[currentStatistics_.status_]++;
    totCount_++;
    trackingStatistics_[t] = currentStatistics_;
    currentStatistics_ = TrackingStatistics();
  }
  int countStatistics(const TrackingStatistics::Status s){
    return cumulativeStatistics_[s] + (int)(currentStatistics_.status_ == s);
  }
  int countStatistics(const TrackingStatistics::Status s, const int n){
    int c = 0;
    auto it = trackingStatistics_.rbegin();
    for(int i=0;i<n-1 && it != trackingStatistics_.rend();++i){
      if(it->second.status_ == s) c++;
      ++it;
    }
    return c + (int)(currentStatistics_.status_ == s);
  }
  double getLocalQuality(const int localRange = 10){
    // Local quality of feature for last inFrames
    int countTracked = 0;
    int countInImage = 0;
    if(currentStatistics_.inFrame_){
      if(currentStatistics_.status_ == TrackingStatistics::TRACKED) countTracked++;
      countInImage++;
    }
    for(auto it = trackingStatistics_.rbegin();countInImage<localRange && it != trackingStatistics_.rend();++it){
      if(it->second.inFrame_){
        if(it->second.status_ == TrackingStatistics::TRACKED) countTracked++;
        countInImage++;
      }
    }
    return static_cast<double>(countTracked)/static_cast<double>(countInImage);
  }
  double getLocalVisibilityQuality(const int localRange = 200){
    int countTot = 0;
    int lastInImage = localRange;
    if(currentStatistics_.inFrame_){
      lastInImage = 0;
    } else {
      countTot++;
      for(auto it = trackingStatistics_.rbegin();countTot<localRange && it != trackingStatistics_.rend();++it){
        if(it->second.inFrame_){
          lastInImage = countTot;
          break;
        }
        countTot++;
      }
    }
    return static_cast<double>(localRange-lastInImage)/static_cast<double>(localRange);
  }
  double getGlobalQuality(){
    const double trackingRatio = static_cast<double>(countStatistics(TrackingStatistics::TRACKED))/static_cast<double>(totCount_-1); // INIT is considered
    return trackingRatio*std::min(static_cast<double>(countStatistics(TrackingStatistics::TRACKED))/100.0,1.0); // param
  }
  bool isGoodFeature(const int localRange = 10, const int localVisibilityRange = 100, const double upper = 0.9, const double lower = 0.1){
    const double globalQuality = getGlobalQuality();
    const double localQuality = getLocalQuality(localRange);
    const double localVisibilityQuality = getLocalVisibilityQuality(localVisibilityRange);
    return localQuality*localVisibilityQuality > upper-(upper-lower)*globalQuality;

    /*
     * Local Quality: how is the tracking within the local range, OUTLIER/NOTFOUND is worse than not inImage
     * Global Quality: how was the overall tracking of the feature (only consider visible)
     * Local Visibility: what was the last frame the feature was seen in
     * Information Quality: what is the nearest neighbour
     * How strong we need place for new features
     */
  }
  MultilevelPatchFeature(){
    idx_ = -1;
    c_ = cv::Point2f(0,0);
    reset();
  }
  MultilevelPatchFeature(int idx,const cv::Point2f& c){
    idx_ = idx;
    c_ = c;
    reset();
  }
  ~MultilevelPatchFeature(){}
  void reset(){
    s_ = 0;
    hasValidPatches_ = false;
    totCount_ = 0;
    cumulativeStatistics_.clear();
    trackingStatistics_.clear();
    currentStatistics_.status_ = TrackingStatistics::INIT; // TODO: rethink
  }
  bool extractPatchesFromImage(const ImagePyramid<n_levels>& pyr){
    bool success = true;
    corners_[0] = cv::Point2f(warpDistance,0);
    corners_[1] = cv::Point2f(0,warpDistance);
    for(unsigned int i=0;i<nLevels_;i++){
      success = success & patches_[i].extractPatchFromImage(pyr.imgs_[i],c_*pow(0.5,i));
    }
    hasValidPatches_ = success;
    return success;
  }
  bool isMultilevelPatchInFrame(const ImagePyramid<n_levels>& pyr,const int hLevel){ // TODO: clean and improve
    const int halfpatch_size = patch_size/2*pow(2,hLevel);
    bool outside = c_.x < halfpatch_size | c_.y < halfpatch_size | c_.x > pyr.imgs_[0].cols-halfpatch_size | c_.y > pyr.imgs_[0].rows-halfpatch_size;
    return !outside;
  }
  bool isMultilevelPatchWithBorderInFrame(const ImagePyramid<n_levels>& pyr,const int hLevel){
    const int halfpatch_size = (patch_size/2+1)*pow(2,hLevel);
    bool outside = c_.x < halfpatch_size | c_.y < halfpatch_size | c_.x > pyr.imgs_[0].cols-halfpatch_size | c_.y > pyr.imgs_[0].rows-halfpatch_size;
    return !outside;
  }
  void computeMultilevelShiTomasiScore(){
    if(hasValidPatches_){
      for(unsigned int i=0;i<nLevels_;i++){
        H_ += pow(0.25,i)*patches_[i].getHessian();
      }
      const float dXX = H_(0,0)/(n_levels*patch_size*patch_size);
      const float dYY = H_(1,1)/(n_levels*patch_size*patch_size);
      const float dXY = H_(0,1)/(n_levels*patch_size*patch_size);
      // Find and return smaller eigenvalue:
      s_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    } else {
      s_ = -1;
    }
  }
  void getSpecificLevelScore(const int level){
    if(patches_[level].hasValidPatch_){
      s_ = patches_[level].getScore();
    } else {
      s_ = -1;
    }
  }
  bool align2D(const ImagePyramid<n_levels>& pyr,cv::Point2f& c, const int level1, const int level2, const bool doWarping){
    for(int level = level1; level <= level2; level++){
      if(!patches_[level].hasValidPatch_) return false;
    }
    Matrix3f Aff; Aff.setIdentity();
    if(doWarping){
      Aff(0,0) = corners_[0].x/warpDistance;
      Aff(1,0) = corners_[0].y/warpDistance;
      Aff(0,1) = corners_[1].x/warpDistance;
      Aff(1,1) = corners_[1].y/warpDistance;
    } // TODO: catch if too distorted, or if warping not required
    const int halfpatch_size = patch_size/2;
    bool converged=false;
    Matrix3f H; H.setZero();
    for(int level = level1; level <= level2; level++){
      patches_[level].computeGradientParameters();
      H(0,0) += pow(0.25,level)*patches_[level].H_(0,0);
      H(0,1) += pow(0.25,level)*patches_[level].H_(0,1);
      H(1,0) += pow(0.25,level)*patches_[level].H_(1,0);
      H(1,1) += pow(0.25,level)*patches_[level].H_(1,1);
      H(2,0) += pow(0.5,level)*patches_[level].H_(2,0);
      H(2,1) += pow(0.5,level)*patches_[level].H_(2,1);
      H(0,2) += pow(0.5,level)*patches_[level].H_(0,2);
      H(1,2) += pow(0.5,level)*patches_[level].H_(1,2);
      H(2,2) += patches_[level].H_(2,2);
    }
    Matrix3f Hinv = H.inverse();
    float mean_diff = 0;
    const int max_iter = 10;

    // Compute pixel location in new image:
    float u = c.x;
    float v = c.y;

    // termination condition
    const float min_update_squared = 0.03*0.03; // TODO: param
    Vector3f update; update.setZero();
    for(int iter = 0; iter<max_iter; ++iter){
      if(isnan(u) || isnan(v)){ // TODO check
        return false;
      }
      Vector3f Jres; Jres.setZero();
      for(int level = level1; level <= level2; level++){
        const int refStep = pyr.imgs_[level].step.p[0];
        const float u_level = u*pow(0.5,level);
        const float v_level = v*pow(0.5,level);

        uint8_t* it_patch = patches_[level].patch_;
        float* it_dx = patches_[level].dx_;
        float* it_dy = patches_[level].dy_;
        if(!doWarping){
          const int u_r = floor(u_level);
          const int v_r = floor(v_level);
          if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= pyr.imgs_[level].cols-halfpatch_size || v_r >= pyr.imgs_[level].rows-halfpatch_size){ // TODO: check limits
            return false;
          }
          // compute interpolation weights
          const float subpix_x = u_level-u_r;
          const float subpix_y = v_level-v_r;
          const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
          const float wTR = subpix_x * (1.0-subpix_y);
          const float wBL = (1.0-subpix_x)*subpix_y;
          const float wBR = subpix_x * subpix_y;

          uint8_t* it_img;        // loop through search_patch, interpolate
          for(int y=0; y<patch_size; ++y){
            it_img = (uint8_t*) pyr.imgs_[level].data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
            for(int x=0; x<patch_size; ++x, ++it_img, ++it_patch, ++it_dx, ++it_dy){
              const float intensity = wTL*it_img[0] + wTR*it_img[1] + wBL*it_img[refStep] + wBR*it_img[refStep+1];
              const float res = intensity - *it_patch + mean_diff;
              Jres[0] -= pow(0.5,level)*res*(*it_dx);
              Jres[1] -= pow(0.5,level)*res*(*it_dy);
              Jres[2] -= res;
            }
          }
        } else {
          for(int y=0; y<patch_size; ++y){ // TODO: renaming
            for(int x=0; x<patch_size; ++x, ++it_patch, ++it_dx, ++it_dy){
              const float dx = x - halfpatch_size;
              const float dy = y - halfpatch_size;
              const float wdx = Aff(0,0)*dx + Aff(0,1)*dy;
              const float wdy = Aff(1,0)*dx + Aff(1,1)*dy;
              const float u_pixel = u_level+wdx;
              const float v_pixel = v_level+wdy;
              const int u_pixel_r = floor(u_pixel);
              const int v_pixel_r = floor(v_pixel);
              if(u_pixel_r < 0 || v_pixel_r < 0 || u_pixel_r >= pyr.imgs_[level].cols-1 || v_pixel_r >= pyr.imgs_[level].rows-1){ // TODO: check limits
                return false;
              }
              const float pixel_subpix_x = u_pixel-u_pixel_r;
              const float pixel_subpix_y = v_pixel-v_pixel_r;
              const float pixel_wTL = (1.0-pixel_subpix_x) * (1.0-pixel_subpix_y);
              const float pixel_wTR = pixel_subpix_x * (1.0-pixel_subpix_y);
              const float pixel_wBL = (1.0-pixel_subpix_x) * pixel_subpix_y;
              const float pixel_wBR = pixel_subpix_x * pixel_subpix_y;
              const uint8_t* pixel_data = (uint8_t*) pyr.imgs_[level].data + v_pixel_r*refStep + u_pixel_r;
              const float pixel_intensity = pixel_wTL*pixel_data[0] + pixel_wTR*pixel_data[1] + pixel_wBL*pixel_data[refStep] + pixel_wBR*pixel_data[refStep+1];
              const float res = pixel_intensity - *it_patch + mean_diff;
              Jres[0] -= pow(0.5,level)*res*(*it_dx);
              Jres[1] -= pow(0.5,level)*res*(*it_dy);
              Jres[2] -= res;
            }
          }
        }
      }

      if(!doWarping){
        update = Hinv * Jres;
      } else {
        update = Aff * Hinv * Jres;
      }
      u += update[0];
      v += update[1];
      mean_diff += update[2];

      if(update[0]*update[0]+update[1]*update[1] < min_update_squared){
        converged=true;
        break;
      }
    }

    c.x = u;
    c.y = v;
    return converged;
  }
  bool align2DSingleLevel(const ImagePyramid<n_levels>& pyr,cv::Point2f& c, const int level, const bool doWarping){
    return align2D(pyr,c,level,level,doWarping);
  }
  bool getLinearAlignEquations(const ImagePyramid<n_levels>& pyr,const cv::Point2f& c, const int level1, const int level2, const bool doWarping,
                               Eigen::MatrixXf& A, Eigen::MatrixXf& b){ // TODO: cleanup
    A.resize((level2-level1+1)*patch_size*patch_size,2);
    b.resize((level2-level1+1)*patch_size*patch_size,1);
    for(int level = level1; level <= level2; level++){
      if(!patches_[level].hasValidPatch_) return false;
      patches_[level].computeGradientParameters();
    }
    Matrix2f Aff,AffInv; Aff.setIdentity();
    if(doWarping){
      Aff(0,0) = corners_[0].x/warpDistance;
      Aff(1,0) = corners_[0].y/warpDistance;
      Aff(0,1) = corners_[1].x/warpDistance;
      Aff(1,1) = corners_[1].y/warpDistance;
      AffInv = Aff.inverse();
    } // TODO: catch if too distorted, or if warping not required, pass matrix here (as member of feature)
    const int halfpatch_size = patch_size/2;
    float mean_diff = 0;
    float mean_diff_dx = 0;
    float mean_diff_dy = 0;

    // Compute pixel location in new image:
    const float u = c.x;
    const float v = c.y;

    for(int level = level1; level <= level2; level++){
      const int refStep = pyr.imgs_[level].step.p[0];
      const float u_level = u*pow(0.5,level);
      const float v_level = v*pow(0.5,level);

      uint8_t* it_patch = patches_[level].patch_;
      float* it_dx = patches_[level].dx_;
      float* it_dy = patches_[level].dy_;
      if(!doWarping){
        const int u_r = floor(u_level);
        const int v_r = floor(v_level);
        if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= pyr.imgs_[level].cols-halfpatch_size || v_r >= pyr.imgs_[level].rows-halfpatch_size){ // TODO: check limits
          return false;
        }
        // compute interpolation weights
        const float subpix_x = u_level-u_r;
        const float subpix_y = v_level-v_r;
        const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
        const float wTR = subpix_x * (1.0-subpix_y);
        const float wBL = (1.0-subpix_x)*subpix_y;
        const float wBR = subpix_x * subpix_y;

        uint8_t* it_img;        // loop through search_patch, interpolate
        for(int y=0; y<patch_size; ++y){
          it_img = (uint8_t*) pyr.imgs_[level].data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
          for(int x=0; x<patch_size; ++x, ++it_img, ++it_patch, ++it_dx, ++it_dy){
            const float intensity = wTL*it_img[0] + wTR*it_img[1] + wBL*it_img[refStep] + wBR*it_img[refStep+1];
            const float res = intensity - *it_patch;
            const float Jx = -pow(0.5,level)*(*it_dx);
            const float Jy = -pow(0.5,level)*(*it_dy);
            mean_diff += res;
            mean_diff_dx += Jx;
            mean_diff_dy += Jy;
            b((level-level1)*patch_size*patch_size+y*patch_size+x,0) = res;
            A((level-level1)*patch_size*patch_size+y*patch_size+x,0) = Jx;
            A((level-level1)*patch_size*patch_size+y*patch_size+x,1) = Jy;
          }
        }
      } else {
        for(int y=0; y<patch_size; ++y){ // TODO: renaming
          for(int x=0; x<patch_size; ++x, ++it_patch, ++it_dx, ++it_dy){
            const float dx = x - halfpatch_size;
            const float dy = y - halfpatch_size;
            const float wdx = Aff(0,0)*dx + Aff(0,1)*dy;
            const float wdy = Aff(1,0)*dx + Aff(1,1)*dy;
            const float u_pixel = u_level+wdx;
            const float v_pixel = v_level+wdy;
            const int u_pixel_r = floor(u_pixel);
            const int v_pixel_r = floor(v_pixel);
            if(u_pixel_r < 0 || v_pixel_r < 0 || u_pixel_r >= pyr.imgs_[level].cols-1 || v_pixel_r >= pyr.imgs_[level].rows-1){ // TODO: check limits
              return false;
            }
            const float pixel_subpix_x = u_pixel-u_pixel_r;
            const float pixel_subpix_y = v_pixel-v_pixel_r;
            const float pixel_wTL = (1.0-pixel_subpix_x) * (1.0-pixel_subpix_y);
            const float pixel_wTR = pixel_subpix_x * (1.0-pixel_subpix_y);
            const float pixel_wBL = (1.0-pixel_subpix_x) * pixel_subpix_y;
            const float pixel_wBR = pixel_subpix_x * pixel_subpix_y;
            const uint8_t* pixel_data = (uint8_t*) pyr.imgs_[level].data + v_pixel_r*refStep + u_pixel_r;
            const float pixel_intensity = pixel_wTL*pixel_data[0] + pixel_wTR*pixel_data[1] + pixel_wBL*pixel_data[refStep] + pixel_wBR*pixel_data[refStep+1];
            const float res = pixel_intensity - *it_patch;
            const float Jx = -pow(0.5,level)*(*it_dx);
            const float Jy = -pow(0.5,level)*(*it_dy);
            const float Jx_warp = Jx*AffInv(0,0)+Jy*AffInv(1,0);
            const float Jy_warp = Jx*AffInv(0,1)+Jy*AffInv(1,1);
            mean_diff += res;
            mean_diff_dx += Jx_warp;
            mean_diff_dy += Jy_warp;
            b((level-level1)*patch_size*patch_size+y*patch_size+x,0) = res;
            A((level-level1)*patch_size*patch_size+y*patch_size+x,0) = Jx_warp;
            A((level-level1)*patch_size*patch_size+y*patch_size+x,1) = Jy_warp;
          }
        }
      }
    }
    mean_diff = mean_diff/static_cast<float>((level2-level1+1)*patch_size*patch_size);
    mean_diff_dx = mean_diff_dx/static_cast<float>((level2-level1+1)*patch_size*patch_size);
    mean_diff_dy = mean_diff_dy/static_cast<float>((level2-level1+1)*patch_size*patch_size);
    b.array() -= mean_diff;
    A.col(0).array() -= mean_diff_dx;
    A.col(1).array() -= mean_diff_dy;
    return true;
  }
  bool getLinearAlignEquationsReduced(const ImagePyramid<n_levels>& pyr,const cv::Point2f& c, const int level1, const int level2, const bool doWarping,
                               Eigen::Matrix2f& A_red, Eigen::Vector2f& b_red){
    bool success = getLinearAlignEquations(pyr,c,level1,level2,doWarping,A_,b_);
    if(success){
      mColPivHouseholderQR_.compute(A_);
      b_red = (mColPivHouseholderQR_.householderQ().inverse()*b_).block<2,1>(0,0);
      A_red = mColPivHouseholderQR_.matrixR().block<2,2>(0,0);
      A_red(1,0) = 0.0;
      A_red = A_red*mColPivHouseholderQR_.colsPermutation();
    }
    return success;
  }
  bool getLinearAlignEquationsReduced(const ImagePyramid<n_levels>& pyr,const cv::Point2f& c, const int level1, const int level2, const bool doWarping,
                               Eigen::Matrix2d& A_red, Eigen::Vector2d& b_red){
    Eigen::Matrix2f A_red_;
    Eigen::Vector2f b_red_;
    bool success = getLinearAlignEquationsReduced(pyr,c,level1,level2,doWarping,A_red_,b_red_);
    if(success){
      A_red = A_red_.cast<double>();
      b_red = b_red_.cast<double>();
    }
    return success;
  }
};

template<int n_levels,int patch_size,int nMax>
class FeatureManager{
 public:
  MultilevelPatchFeature<n_levels,patch_size> features_[nMax];
  std::unordered_set<unsigned int> validSet_;
  std::unordered_set<unsigned int> invalidSet_;
  std::vector<MultilevelPatchFeature<n_levels,patch_size>> candidates_;
  int maxIdx_;
  FeatureManager(){
    maxIdx_ = 0;
    for(unsigned int i=0;i<nMax;i++){
      invalidSet_.insert(i);
    }
  }
  int addFeature(const MultilevelPatchFeature<n_levels,patch_size>& feature){
    if(invalidSet_.empty()){
      std::cout << "Feature Manager: maximal number of feature reached" << std::endl;
      return -1;
    } else {
      unsigned int ind = *(invalidSet_.begin());
      invalidSet_.erase(ind);
      validSet_.insert(ind);
      features_[ind] = feature;
      features_[ind].idx_ = maxIdx_++;
      return ind;
    }
  }
  void removeFeature(unsigned int ind){
      validSet_.erase(ind);
      invalidSet_.insert(ind);
  }
  ~FeatureManager(){}
  void selectCandidates(std::vector<cv::Point2f>& detected_keypoints){ // TODO: add corner motion dependency
    float d2;
    constexpr float t2 = patch_size*patch_size; // TODO: param
    bool add;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for (auto it = detected_keypoints.begin(); it != detected_keypoints.end(); ++it) {
      add = true;
      for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
        mpFeature = &features_[*it_f];
        d2 = pow(it->x-mpFeature->c_.x,2) + pow(it->y-mpFeature->c_.y,2); // TODO: check inFrame
        if(d2 < t2){
          add = false;
        }

      }
      if(add){
        candidates_.emplace_back(0,*it);
      }
    }
  }
  void extractCandidatePatchesFromImage(const ImagePyramid<n_levels>& pyr){
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
      it->extractPatchesFromImage(pyr);
    }
  }
  void computeCandidatesScore(int mode){
    if(mode < 0){
      for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
        it->computeMultilevelShiTomasiScore();
      }
    } else {
      for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
        it->getSpecificLevelScore(mode);
      }
    }
  }
  std::unordered_set<unsigned int> addBestCandidates(const int maxN,cv::Mat& drawImg, const int nDetectionBuckets, const double scoreDetectionExponent,
                                                     const double penaltyDistance, const double zeroDistancePenalty, const bool requireMax){
    std::unordered_set<unsigned int> newSet;
    float maxScore = -1.0;
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){;
      if(it->s_ > maxScore) maxScore = it->s_;
    }

    // Make buckets and fill based on score
    std::vector<std::unordered_set<MultilevelPatchFeature<n_levels,patch_size>*>> buckets(nDetectionBuckets,std::unordered_set<MultilevelPatchFeature<n_levels,patch_size>*>());
    unsigned int newBucketID;
    for (auto it_cand = candidates_.begin(); it_cand != candidates_.end(); ++it_cand) {
      if(it_cand->s_ > 0.0){
        newBucketID = std::ceil((nDetectionBuckets-1)*(pow(it_cand->s_/maxScore,static_cast<float>(scoreDetectionExponent))));
        if(newBucketID>nDetectionBuckets-1) newBucketID = nDetectionBuckets-1;
        buckets[newBucketID].insert(&(*it_cand));
      }
    }

    // Move buckets based on current features
    double d2;
    double t2 = pow(penaltyDistance,2);
    bool doDelete;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      for (unsigned int bucketID = 1;bucketID < nDetectionBuckets;bucketID++) {
        for (auto it_cand = buckets[bucketID].begin();it_cand != buckets[bucketID].end();) {
          doDelete = false;
          d2 = std::pow(mpFeature->c_.x - (*it_cand)->c_.x,2) + std::pow(mpFeature->c_.y - (*it_cand)->c_.y,2);
          if(d2<t2){
            newBucketID = std::max((int)(bucketID - (t2-d2)/t2*zeroDistancePenalty),0);
            if(bucketID != newBucketID){
              buckets[newBucketID].insert(*it_cand);
              doDelete = true;
            }
          }
          if(doDelete){
            buckets[bucketID].erase(it_cand++);
          } else {
            ++it_cand;
          }
        }
      }
    }

    // Incrementally add features and update candidate buckets
    MultilevelPatchFeature<n_levels,patch_size>* mpNewFeature;
    int addedCount = 0;
    for (int bucketID = nDetectionBuckets-1;bucketID >= 0+static_cast<int>(!requireMax);bucketID--) {
      while(!buckets[bucketID].empty() && addedCount < maxN) {
        mpNewFeature = *(buckets[bucketID].begin());
        buckets[bucketID].erase(mpNewFeature);
        int ind = addFeature(*mpNewFeature);
        if(ind >= 0){
          newSet.insert(ind);
        }
        addedCount++;
        for (unsigned int bucketID2 = 1;bucketID2 <= bucketID;bucketID2++) {
          for (auto it_cand = buckets[bucketID2].begin();it_cand != buckets[bucketID2].end();) {
            doDelete = false;
            d2 = std::pow(mpNewFeature->c_.x - (*it_cand)->c_.x,2) + std::pow(mpNewFeature->c_.y - (*it_cand)->c_.y,2);
            if(d2<t2){
              newBucketID = std::max((int)(bucketID2 - (t2-d2)/t2*zeroDistancePenalty),0);
              if(bucketID2 != newBucketID){
                buckets[newBucketID].insert(*it_cand);
                doDelete = true;
              }
            }
            if(doDelete){
              buckets[bucketID2].erase(it_cand++);
            } else {
              ++it_cand;
            }
          }
        }
      }
    }
    return newSet;
  }
  void alignFeaturesCom(const ImagePyramid<n_levels>& pyr,cv::Mat& drawImg,const int start_level,const int end_level, const int num_seq, const bool doWarping){ // TODO: clean
    cv::Point2f c_new;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      if(mpFeature->currentStatistics_.inFrame_){
        c_new = mpFeature->c_;
        mpFeature->currentStatistics_.status_ = TrackingStatistics::FOUND;
        for(int level = end_level+num_seq;level>=end_level;--level){
          if(!mpFeature->align2D(pyr,c_new,level,start_level,doWarping)){
            mpFeature->currentStatistics_.status_ = TrackingStatistics::NOTFOUND;
            break;
          }
        }
        if(mpFeature->currentStatistics_.status_ == TrackingStatistics::FOUND){
          mpFeature->c_ = c_new;
        }
      }
    }
  }
  void alignFeaturesSingle(const ImagePyramid<n_levels>& pyr,cv::Mat& drawImg,const int start_level,const int end_level, const bool doWarping){
    cv::Point2f c_new;
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      if(mpFeature->currentStatistics_.inFrame_){
        c_new = mpFeature->c_;
        mpFeature->currentStatistics_.status_ = TrackingStatistics::FOUND;
        for(int level = start_level;level>=end_level;--level){
          if(!mpFeature->align2DSingleLevel(pyr,c_new,level,doWarping) && level==end_level){
            mpFeature->currentStatistics_.status_ = TrackingStatistics::NOTFOUND;
            break;
          }
        }
        if(mpFeature->currentStatistics_.status_ == TrackingStatistics::FOUND){
          mpFeature->c_ = c_new;
        }
      }
    }
  }
  void drawCandidates(cv::Mat& drawImg){
    for(auto it = candidates_.begin(); it != candidates_.end(); ++it){
      cv::circle(drawImg,it->c_, 3, cv::Scalar(0, 0, 0), -1, 8);
    }
  }
  void drawFeatures(cv::Mat& drawImg){
    MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
    for(auto it_f = validSet_.begin(); it_f != validSet_.end();++it_f){
      mpFeature = &features_[*it_f];
      cv::circle(drawImg,mpFeature->c_, 3, cv::Scalar(0, 0, 0), -1, 8);
    }
  }
};

template <int n_levels>
void DetectFastCorners(const ImagePyramid<n_levels>& pyr, std::vector<cv::Point2f>& detected_keypoints,int level) {
  std::vector<cv::KeyPoint> keypoints;
  cv::FastFeatureDetector feature_detector_fast(10, true); // TODO param
  feature_detector_fast.detect(pyr.imgs_[level], keypoints);
  detected_keypoints.reserve(keypoints.size());
  for (auto it = keypoints.cbegin(), end = keypoints.cend(); it != end; ++it) {
    detected_keypoints.emplace_back(it->pt.x*pow(2.0,level), it->pt.y*pow(2.0,level));
  }
}

}


#endif /* COMMON_VISION_HPP_ */
