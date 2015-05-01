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

#ifndef ROVIO_COMMON_VISION_HPP_
#define ROVIO_COMMON_VISION_HPP_

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_set>
#include <Eigen/Eigenvalues>
#include "lightweight_filtering/State.hpp"
#include "rovio/Camera.hpp"

namespace rovio{

enum MatchingStatus{
  NOTMATCHED,
  NOTFOUND,
  FOUND
};
enum TrackingStatus{
  NOTTRACKED,
  FAILED,
  TRACKED
};
struct Status{
  bool inFrame_;
  MatchingStatus matchingStatus_;
  TrackingStatus trackingStatus_;
  Status(){
    inFrame_ = false;
    matchingStatus_ = NOTMATCHED;
    trackingStatus_ = NOTTRACKED;
  }
};

void halfSample(const cv::Mat& imgIn,cv::Mat& imgOut){
  imgOut.create(imgIn.rows/2,imgIn.cols/2,imgIn.type());
  const int refStepIn = imgIn.step.p[0];
  const int refStepOut = imgOut.step.p[0];
  uint8_t* imgPtrInTop;
  uint8_t* imgPtrInBot;
  uint8_t* imgPtrOut;
  for(int y=0; y<imgOut.rows; ++y){
    imgPtrInTop =  imgIn.data + 2*y*refStepIn;
    imgPtrInBot =  imgIn.data + (2*y+1)*refStepIn;
    imgPtrOut = imgOut.data + y*refStepOut;
    for(int x=0; x<imgOut.cols; ++x, ++imgPtrOut, imgPtrInTop += 2, imgPtrInBot += 2)
    {
      *imgPtrOut = (imgPtrInTop[0]+imgPtrInTop[1]+imgPtrInBot[0]+imgPtrInBot[1])/4;
    }
  }
}

template<int n_levels>
class ImagePyramid{
 public:
  ImagePyramid(){};
  ~ImagePyramid(){};
  cv::Mat imgs_[n_levels];
  cv::Point2f centers_[n_levels];
  void computeFromImage(const cv::Mat& img, const bool useCv = false){
    img.copyTo(imgs_[0]);
    centers_[0] = cv::Point2f(0,0);
    for(int i=1; i<n_levels; ++i){
      if(!useCv){
        halfSample(imgs_[i-1],imgs_[i]); // TODO: check speed
        centers_[i].x = centers_[i-1].x-pow(0.5,2-i)*(float)(imgs_[i-1].rows%2);
        centers_[i].y = centers_[i-1].y-pow(0.5,2-i)*(float)(imgs_[i-1].cols%2);
      } else {
        cv::pyrDown(imgs_[i-1],imgs_[i],cv::Size(imgs_[i-1].cols/2, imgs_[i-1].rows/2));
        centers_[i].x = centers_[i-1].x-pow(0.5,2-i)*(float)((imgs_[i-1].rows%2)+1);
        centers_[i].y = centers_[i-1].y-pow(0.5,2-i)*(float)((imgs_[i-1].cols%2)+1);
      }
    }
  }
  ImagePyramid<n_levels>& operator=(const ImagePyramid<n_levels> &rhs) {
    for(unsigned int i=0;i<n_levels;i++){
      rhs.imgs_[i].copyTo(imgs_[i]);
      centers_[i] = rhs.centers_[i];
    }
    return *this;
  }
};

struct PixelCorners{
  cv::Point2f corners_[2];
  cv::Point2f& operator[](unsigned int i){
    return corners_[i];
  };
  const cv::Point2f& operator[](unsigned int i) const{
    return corners_[i];
  };
};

struct BearingCorners{
  Eigen::Vector2d corners_[2];
  Eigen::Vector2d& operator[](unsigned int i){
    return corners_[i];
  };
  const Eigen::Vector2d& operator[](unsigned int i) const{
    return corners_[i];
  };
};

template<int size>
class Patch {
 public:
  float patch_[size*size] __attribute__ ((aligned (16)));
  float patchWithBorder_[(size+2)*(size+2)] __attribute__ ((aligned (16)));
  float dx_[size*size] __attribute__ ((aligned (16)));
  float dy_[size*size] __attribute__ ((aligned (16)));
  Eigen::Matrix3f H_;
  float s_;
  bool validGradientParameters_;
  Patch(){
    static_assert(size%2==0,"Patch size must be a multiple of 2");
    validGradientParameters_ = false;
    s_ = 0.0;
  }
  ~Patch(){}
  void computeGradientParameters(){
    H_.setZero();
    const int refStep = size+2;
    float* it_dx = dx_;
    float* it_dy = dy_;
    float* it;
    Eigen::Vector3f J;
    for(int y=0; y<size; ++y){
      it = patchWithBorder_ + (y+1)*refStep + 1;
      for(int x=0; x<size; ++x, ++it, ++it_dx, ++it_dy){
        J[0] = 0.5 * (it[1] - it[-1]);
        J[1] = 0.5 * (it[refStep] - it[-refStep]);
        J[2] = 1;
        *it_dx = J[0];
        *it_dy = J[1];
        H_ += J*J.transpose();
      }
    }
    const float dXX = H_(0,0)/(size*size);
    const float dYY = H_(1,1)/(size*size);
    const float dXY = H_(0,1)/(size*size);
    // Find and return smaller eigenvalue:
    s_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    validGradientParameters_ = true;
  }
  void extractPatchFromPatchWithBorder(){
    float* it_patch = patch_;
    float* it_patchWithBorder;
    for(int y=1; y<size+1; ++y, it_patch += size){
      it_patchWithBorder = patchWithBorder_ + y*(size+2) + 1;
      for(int x=0; x<size; ++x)
        it_patch[x] = it_patchWithBorder[x];
    }
  }
  float getScore(){
    if(!validGradientParameters_) computeGradientParameters();
    return s_;
  }
  Eigen::Matrix3f getHessian(){
    if(!validGradientParameters_) computeGradientParameters();
    return H_;
  }
  void drawPatch(cv::Mat& drawImg,const cv::Point2i& c,int stretch = 1,const bool withBorder = false){
    const int refStep = drawImg.step.p[0];
    uint8_t* img_ptr;
    float* it_patch;
    if(withBorder){
      it_patch = patchWithBorder_;
    } else {
      it_patch = patch_;
    }
    for(int y=0; y<size+2*(int)withBorder; ++y, it_patch += size+2*(int)withBorder){
      img_ptr = (uint8_t*) drawImg.data + (c.y+y*stretch)*refStep + c.x;
      for(int x=0; x<size+2*(int)withBorder; ++x)
        for(int i=0;i<stretch;++i){
          for(int j=0;j<stretch;++j){
            img_ptr[x*stretch+i*refStep+j] = (uint8_t)(it_patch[x]);
          }
        }
    }
  }
};

template<int size>
bool isPatchInFrame(const Patch<size>& patch,const cv::Mat& img,const cv::Point2f& c,const bool withBorder = false){
  const int halfpatch_size = size/2+(int)withBorder;
  if(c.x < halfpatch_size || c.y < halfpatch_size || c.x > img.cols-halfpatch_size || c.y > img.rows-halfpatch_size){
    return false;
  } else {
    return true;
  }
}

template<int size>
bool isWarpedPatchInFrame(const Patch<size>& patch,const cv::Mat& img,const cv::Point2f& c,const Eigen::Matrix2f& aff,const bool withBorder = false){
  const int halfpatch_size = size/2+(int)withBorder;
  for(int x = 0;x<2;x++){
    for(int y = 0;y<2;y++){
      const float dx = halfpatch_size*(2*x-1);
      const float dy = halfpatch_size*(2*y-1);
      const float wdx = aff(0,0)*dx + aff(0,1)*dy;
      const float wdy = aff(1,0)*dx + aff(1,1)*dy;
      const float c_x = c.x + wdx;
      const float c_y = c.y + wdy;
      if(c_x < 0 || c_y < 0 || c_x > img.cols || c_y > img.rows){
        return false;
      }
    }
  }
  return true;
}

template<int size>
void extractPatchFromImage(Patch<size>& patch,const cv::Mat& img,const cv::Point2f& c, const bool withBorder = true){
  assert(isPatchInFrame(patch,img,c,withBorder));
  const int halfpatch_size = size/2+(int)withBorder;
  const int refStep = img.step.p[0];
  const int u_r = floor(c.x);
  const int v_r = floor(c.y);

  // compute interpolation weights
  const float subpix_x = c.x-u_r;
  const float subpix_y = c.y-v_r;
  const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
  const float wTR = subpix_x * (1.0-subpix_y);
  const float wBL = (1.0-subpix_x)*subpix_y;
  const float wBR = subpix_x * subpix_y;

  // Extract Patch
  float* patch_ptr;
  if(withBorder){
    patch_ptr = patch.patchWithBorder_;
  } else {
    patch_ptr = patch.patch_;
  }
  uint8_t* img_ptr;
  for(int y=0; y<2*halfpatch_size; ++y){
    img_ptr = (uint8_t*) img.data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
    for(int x=0; x<2*halfpatch_size; ++x, ++img_ptr, ++patch_ptr)
    {
      *patch_ptr = wTL*img_ptr[0];
      if(subpix_x > 0) *patch_ptr += wTR*img_ptr[1];
      if(subpix_y > 0) *patch_ptr += wBL*img_ptr[refStep];
      if(subpix_x > 0 && subpix_y > 0) *patch_ptr += wBR*img_ptr[refStep+1];
    }
  }
  if(withBorder){
    patch.extractPatchFromPatchWithBorder();
  }
}

template<int size>
void extractWarpedPatchFromImage(Patch<size>& patch,const cv::Mat& img,const cv::Point2f& c,const Eigen::Matrix2f& aff, const bool withBorder = true){
  assert(isWarpedPatchInFrame(patch,img,c,aff,withBorder));
  const int halfpatch_size = size/2+(int)withBorder;
  const int refStep = img.step.p[0];
  uint8_t* img_ptr;
  float* patch_ptr;
  if(withBorder){
    patch_ptr = patch.patchWithBorder_;
  } else {
    patch_ptr = patch.patch_;
  }
  for(int y=0; y<2*halfpatch_size; ++y){
    for(int x=0; x<2*halfpatch_size; ++x, ++patch_ptr){
      const float dx = x - halfpatch_size + 0.5;
      const float dy = y - halfpatch_size + 0.5;
      const float wdx = aff(0,0)*dx + aff(0,1)*dy;
      const float wdy = aff(1,0)*dx + aff(1,1)*dy;
      const float u_pixel = c.x+wdx - 0.5;
      const float v_pixel = c.y+wdy - 0.5;
      const int u_r = floor(u_pixel);
      const int v_r = floor(v_pixel);
      const float subpix_x = u_pixel-u_r;
      const float subpix_y = v_pixel-v_r;
      const float wTL = (1.0-subpix_x) * (1.0-subpix_y);
      const float wTR = subpix_x * (1.0-subpix_y);
      const float wBL = (1.0-subpix_x) * subpix_y;
      const float wBR = subpix_x * subpix_y;
      img_ptr = (uint8_t*) img.data + v_r*refStep + u_r;
      *patch_ptr = wTL*img_ptr[0];
      if(subpix_x > 0) *patch_ptr += wTR*img_ptr[1];
      if(subpix_y > 0) *patch_ptr += wBL*img_ptr[refStep];
      if(subpix_x > 0 && subpix_y > 0) *patch_ptr += wBR*img_ptr[refStep+1];

    }
  }
  if(withBorder){
    patch.extractPatchFromPatchWithBorder();
  }
}

class FeatureCoordinates{
 public:
  cv::Point2f c_;
  bool valid_c_;
  LWF::NormalVectorElement nor_;
  bool valid_nor_;
  const Camera* mpCamera_;
  double sigma1_;
  double sigma2_;
  double sigmaAngle_;
  Eigen::EigenSolver<Eigen::Matrix2d> es_;
  FeatureCoordinates(){
    mpCamera_ == nullptr;
    resetCoordinates();
  }
  FeatureCoordinates(const Camera* mpCamera): mpCamera_(mpCamera){
    resetCoordinates();
  }
  void resetCoordinates(){
    valid_c_ = false;
    valid_nor_ = false;
    sigma1_ = 0.0;
    sigma2_ = 0.0;
    sigmaAngle_ = 0.0;
  }
  const cv::Point2f& get_c(){
    if(!valid_c_){
      if(valid_nor_ && mpCamera_->bearingToPixel(nor_,c_)){
        valid_c_ = true;
      } else {
        std::cout << "ERROR: No valid coordinate data!" << std::endl;
      }
    }
    return c_;
  }
  const LWF::NormalVectorElement& get_nor(){
    if(!valid_nor_){
      if(valid_c_ && mpCamera_->pixelToBearing(c_,nor_)){
        valid_nor_ = true;
      } else {
        std::cout << "ERROR: No valid coordinate data!" << std::endl;
      }
    }
    return nor_;
  }
  void set_c(const cv::Point2f& c){
    c_ = c;
    valid_c_ = true;
    valid_nor_ = false;
  }
  void set_nor(const LWF::NormalVectorElement& nor){
    nor_ = nor;
    valid_nor_ = true;
    valid_c_ = false;
  }
  bool isInFront(){
    return valid_c_ || (valid_nor_ && nor_.getVec()[2] > 0);
  }
  void setSigmaFromCov(const Eigen::Matrix2d& cov){
    es_.compute(cov);
    sigmaAngle_ = std::atan2(es_.eigenvectors()(1,0).real(),es_.eigenvectors()(0,0).real());
    sigma1_ = sqrt(es_.eigenvalues()(0).real());
    sigma2_ = sqrt(es_.eigenvalues()(1).real());
    if(sigma1_<sigma2_){
      const double temp = sigma1_;
      sigma1_ = sigma2_;
      sigma2_ = temp;
      sigmaAngle_ += 0.5*M_PI;
    }
  }
};

static void drawPoint(cv::Mat& drawImg, const FeatureCoordinates& C, const cv::Scalar& color){
  cv::Size size(2,2);
  cv::ellipse(drawImg,C.c_,size,0,0,360,color,-1,8,0);
}
static void drawEllipse(cv::Mat& drawImg, const FeatureCoordinates& C, const cv::Scalar& color, double scaleFactor = 2.0){
  drawPoint(drawImg,C,color);
  cv::ellipse(drawImg,C.c_,cv::Size(std::max(static_cast<int>(scaleFactor*C.sigma1_+0.5),1),std::max(static_cast<int>(scaleFactor*C.sigma2_+0.5),1)),C.sigmaAngle_*180/M_PI,0,360,color,1,8,0);
}
static void drawLine(cv::Mat& drawImg, const FeatureCoordinates& C1, const FeatureCoordinates& C2, const cv::Scalar& color, int thickness = 2){
  cv::line(drawImg,C1.c_,C2.c_,color,thickness);
}
static void drawText(cv::Mat& drawImg, const FeatureCoordinates& C, const std::string& s, const cv::Scalar& color){
  cv::putText(drawImg,s,C.c_,cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
}

template<int n_levels,int patch_size>
class MultilevelPatchFeature: public FeatureCoordinates{
 public:
  static const int nLevels_ = n_levels;
  Patch<patch_size> patches_[nLevels_];
  bool isValidPatch_[nLevels_];
  static constexpr float warpDistance_ = static_cast<float>(patch_size);
  PixelCorners pixelCorners_;
  bool valid_pixelCorners_;
  BearingCorners bearingCorners_;
  bool valid_bearingCorners_;
  Eigen::Matrix2f affineTransform_;
  bool valid_affineTransform_;
  int idx_;
  double initTime_;
  double currentTime_;
  Eigen::Matrix3f H_;
  float s_;
  int totCount_;
  Eigen::MatrixXf A_;
  Eigen::MatrixXf b_;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXf> mColPivHouseholderQR_;

  FeatureCoordinates log_previous_;
  FeatureCoordinates log_prediction_;
  FeatureCoordinates log_predictionC0_;
  FeatureCoordinates log_predictionC1_;
  FeatureCoordinates log_predictionC2_;
  FeatureCoordinates log_predictionC3_;
  FeatureCoordinates log_meas_;
  FeatureCoordinates log_current_;

  Status status_;
  std::map<MatchingStatus,int> cumulativeMatchingStatus_;
  std::map<TrackingStatus,int> cumulativeTrackingStatus_;
  int inFrameCount_;
  std::map<double,Status> statistics_;

  MultilevelPatchFeature(){
    reset();
  }
  MultilevelPatchFeature(const Camera* mpCamera): FeatureCoordinates(mpCamera){
    reset();
  }
  ~MultilevelPatchFeature(){}
  void setCamera(const Camera* mpCamera){
    mpCamera_ = mpCamera;
  }
  void reset(const int idx = -1, const double initTime = 0.0){
    resetCoordinates();
    idx_ = idx;
    initTime_ = initTime;
    currentTime_ = initTime;
    valid_pixelCorners_ = false;
    valid_bearingCorners_ = false;
    valid_affineTransform_ = false;
    H_.setIdentity();
    s_ = 0;
    totCount_ = 0;
    inFrameCount_ = 0;
    statistics_.clear();
    cumulativeMatchingStatus_.clear();
    cumulativeTrackingStatus_.clear();
    status_ = Status();
    for(unsigned int i = 0;i<nLevels_;i++){
      isValidPatch_[i] = false;
    }
  }
  const PixelCorners& get_pixelCorners(){
    if(!valid_pixelCorners_){
      cv::Point2f tempPixel;
      LWF::NormalVectorElement tempNormal;
      if(valid_bearingCorners_){
        for(unsigned int i=0;i<2;i++){
          get_nor().boxPlus(bearingCorners_[i],tempNormal);
          if(!mpCamera_->bearingToPixel(tempNormal,tempPixel)){
            std::cout << "ERROR: Problem during bearing corner to pixel mapping!" << std::endl;
          }
          pixelCorners_[i] = tempPixel - get_c();
        }
        valid_pixelCorners_ = true;
      } else if(valid_affineTransform_) {
        for(unsigned int i=0;i<2;i++){
          pixelCorners_[i].x = affineTransform_(0,i)*warpDistance_;
          pixelCorners_[i].y = affineTransform_(1,i)*warpDistance_;
        }
        valid_pixelCorners_ = true;
      } else {
        std::cout << "ERROR: No valid corner data!" << std::endl;
      }
    }
    return pixelCorners_;
  }
  const BearingCorners& get_bearingCorners(){
    if(!valid_bearingCorners_){
      cv::Point2f tempPixel;
      LWF::NormalVectorElement tempNormal;
      get_pixelCorners();
      for(unsigned int i=0;i<2;i++){
        tempPixel = get_c()+pixelCorners_[i];
        if(!mpCamera_->pixelToBearing(tempPixel,tempNormal)){
          std::cout << "ERROR: Problem during pixel corner to bearing mapping!" << std::endl;
        }
        tempNormal.boxMinus(get_nor(),bearingCorners_[i]);
      }
      valid_bearingCorners_ = true;
    }
    return bearingCorners_;
  }
  const Eigen::Matrix2f& get_affineTransform(){
    if(!valid_affineTransform_){
      cv::Point2f tempPixel;
      LWF::NormalVectorElement tempNormal;
      get_pixelCorners();
      for(unsigned int i=0;i<2;i++){
        affineTransform_(0,i) = pixelCorners_[i].x/warpDistance_;
        affineTransform_(1,i) = pixelCorners_[i].y/warpDistance_;
      }
      valid_affineTransform_ = true;
    }
    return affineTransform_;
  }
  void set_pixelCorners(const PixelCorners& pixelCorners){
    pixelCorners_ = pixelCorners;
    valid_pixelCorners_ = true;
    valid_bearingCorners_ = false;
    valid_affineTransform_ = false;
  }
  void set_bearingCorners(const BearingCorners& bearingCorners){
    bearingCorners_ = bearingCorners;
    valid_bearingCorners_ = true;
    valid_pixelCorners_ = false;
    valid_affineTransform_ = false;
  }
  void set_affineTransfrom(const Eigen::Matrix2f& affineTransform){
    affineTransform_ = affineTransform;
    valid_affineTransform_ = true;
    valid_bearingCorners_ = false;
    valid_pixelCorners_ = false;
  }
  void increaseStatistics(const double& currentTime){
    if(currentTime <= currentTime_) std::cout << "ERROR: timing is not incremental" << std::endl;
    cumulativeMatchingStatus_[status_.matchingStatus_]++;
    cumulativeTrackingStatus_[status_.trackingStatus_]++;
    totCount_++;
    if(status_.inFrame_) inFrameCount_++;
    statistics_[currentTime_] = status_;
    currentTime_ = currentTime;
    status_ = Status();
  }
  int countMatchingStatistics(const MatchingStatus s){
    return cumulativeMatchingStatus_[s] + (int)(status_.matchingStatus_ == s);
  }
  int countTrackingStatistics(const TrackingStatus s){
    return cumulativeTrackingStatus_[s] + (int)(status_.trackingStatus_ == s);
  }
  int countMatchingStatistics(const MatchingStatus s, const int n){
    int c = 0;
    auto it = statistics_.rbegin();
    for(int i=0;i<n-1 && it != statistics_.rend();++i){
      if(it->second.matchingStatus_ == s) c++;
      ++it;
    }
    return c + (int)(status_.matchingStatus_ == s);
  }
  int countTrackingStatistics(const TrackingStatus s, const int n){
    int c = 0;
    auto it = statistics_.rbegin();
    for(int i=0;i<n-1 && it != statistics_.rend();++i){
      if(it->second.trackingStatus_ == s) c++;
      ++it;
    }
    return c + (int)(status_.trackingStatus_ == s);
  }
  int countTot(){
    return totCount_+1;
  }
  int countTotInFrame(){
    return inFrameCount_+(int)(status_.inFrame_);
  }
  double getLocalQuality(const int localRange = 10){
    // Local quality of feature for last "inFrames"
    int countTracked = 0;
    int countInFrame = 0;
    if(status_.inFrame_){
      if(status_.trackingStatus_ == TRACKED) countTracked++;
      countInFrame++;
    }
    for(auto it = statistics_.rbegin();countInFrame<localRange && it != statistics_.rend();++it){
      if(it->second.inFrame_){
        if(it->second.trackingStatus_ == TRACKED) countTracked++;
        countInFrame++;
      }
    }
    if(countInFrame == 0){
      return 0;
    } else {
      return static_cast<double>(countTracked)/static_cast<double>(countInFrame);
    }
  }
  double getLocalVisibilityQuality(const int localRange = 200){
    int countTot = 0;
    int countInFrame = 0;
    countTot++;
    if(status_.inFrame_) countInFrame++;
    for(auto it = statistics_.rbegin();countTot<localRange && it != statistics_.rend();++it){
      countTot++;
      if(it->second.inFrame_) countInFrame++;
    }
    return static_cast<double>(countInFrame)/static_cast<double>(countTot);
  }
  double getGlobalQuality(const int frameCountRef = 100){
    const double trackingRatio = static_cast<double>(countTrackingStatistics(TRACKED))/static_cast<double>(countTot());
    return trackingRatio*std::min(static_cast<double>(countTot())/frameCountRef,1.0);
  }
  bool isGoodFeature(const int localRange = 10, const int localVisibilityRange = 100, const double upper = 0.8, const double lower = 0.1){
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
  void computeMultilevelShiTomasiScore(const int l1 = 0, const int l2 = nLevels_-1){
    H_.setZero();
    int count = 0;
    for(int i=l1;i<=l2;i++){
      if(isValidPatch_[i]){
        H_ += pow(0.25,i)*patches_[i].getHessian();
        count++;
      }
    }
    if(count > 0){
      const float dXX = H_(0,0)/(count*patch_size*patch_size);
      const float dYY = H_(1,1)/(count*patch_size*patch_size);
      const float dXY = H_(0,1)/(count*patch_size*patch_size);
      s_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    } else {
      s_ = -1;
    }
  }
  void getSpecificLevelScore(const int l){
    if(isValidPatch_[l]){
      s_ = patches_[l].getScore();
    } else {
      s_ = -1;
    }
  }
  void drawMultilevelPatch(cv::Mat& drawImg,const cv::Point2i& c,int stretch = 1,const bool withBorder = false){
    for(int l=nLevels_-1;l>=0;l--){
      if(isValidPatch_[l]){
        cv::Point2i corner = cv::Point2i((patch_size/2+(int)withBorder)*(pow(2,nLevels_-1)-pow(2,l)),(patch_size/2+(int)withBorder)*(pow(2,nLevels_-1)-pow(2,l)));
        patches_[l].drawPatch(drawImg,c+corner,stretch*pow(2,l),withBorder);
      }
    }
  }
};

template<int n_levels,int patch_size>
bool isMultilevelPatchInFrame(MultilevelPatchFeature<n_levels,patch_size>& mlp,const ImagePyramid<n_levels>& pyr, const int l = n_levels-1,const bool withBorder = false, const bool doWarping = false){
  if(!mlp.isInFront()) return false;
  const cv::Point2f c = levelTranformCoordinates(mlp.get_c(),pyr,0,l);
  if(!doWarping){
    return isPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c,withBorder);
  } else {
    return isWarpedPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c,mlp.get_affineTransform(),withBorder);
  }
}

template<int n_levels,int patch_size>
void extractMultilevelPatchFromImage(MultilevelPatchFeature<n_levels,patch_size>& mlp,const ImagePyramid<n_levels>& pyr, const int l = n_levels-1, const bool withBorder = true, const bool doWarping = false){
  for(unsigned int i=0;i<=l;i++){
    const cv::Point2f c = levelTranformCoordinates(mlp.get_c(),pyr,0,i);
    if(!doWarping){
      mlp.isValidPatch_[i] = isPatchInFrame(mlp.patches_[i],pyr.imgs_[i],c,withBorder);
      if(mlp.isValidPatch_[i]){
        extractPatchFromImage(mlp.patches_[i],pyr.imgs_[i],c,withBorder);
      }
    } else {
      mlp.isValidPatch_[i] = isWarpedPatchInFrame(mlp.patches_[i],pyr.imgs_[i],c,mlp.get_affineTransform(),withBorder);
      if(mlp.isValidPatch_[i]){
        extractWarpedPatchFromImage(mlp.patches_[i],pyr.imgs_[i],c,mlp.get_affineTransform(),withBorder);
      }
    }
  }
  if(!doWarping) mlp.set_affineTransfrom(Eigen::Matrix2f::Identity());
}

template<int n_levels>
cv::Point2f levelTranformCoordinates(const cv::Point2f c,const ImagePyramid<n_levels>& pyr,const int l1, const int l2){
  assert(l1<n_levels && l2<n_levels);
  return (pyr.centers_[l1]-pyr.centers_[l2])*pow(0.5,l2)+c*pow(0.5,l2-l1);
}

template<int n_levels,int patch_size>
bool getLinearAlignEquations(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr, const int l1, const int l2,
                             const bool doWarping, Eigen::MatrixXf& A, Eigen::MatrixXf& b){
  Eigen::Matrix2f aff;
  if(doWarping){
    aff = mlp.get_affineTransform(); // TODO: catch if too distorted
  } else {
    aff.setIdentity();
  }
  int numLevel = 0;
  for(int l = l1; l <= l2; l++){
    const cv::Point2f c_level = levelTranformCoordinates(mlp.get_c(),pyr,0,l);
    if(mlp.isValidPatch_[l] &&
        (doWarping || isPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c_level,false)) &&
        (!doWarping || isWarpedPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c_level,aff,false))){
      numLevel++;
      mlp.patches_[l].computeGradientParameters();
    }
  }
  if(numLevel==0){
    return false;
  }
  A.resize(numLevel*patch_size*patch_size,2);
  b.resize(numLevel*patch_size*patch_size,1);
  Eigen::Matrix2f affInv = aff.inverse();
  const int halfpatch_size = patch_size/2;
  float mean_diff = 0;
  float mean_diff_dx = 0;
  float mean_diff_dy = 0;
  Patch<patch_size> extractedPatch;

  int count = 0;
  for(int l = l1; l <= l2; l++, count++){
    const cv::Point2f c_level = levelTranformCoordinates(mlp.get_c(),pyr,0,l);
    if(mlp.isValidPatch_[l] &&
        (doWarping || isPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c_level,false)) &&
        (!doWarping || isWarpedPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c_level,aff,false))){
      const int refStep = pyr.imgs_[l].step.p[0];

      float* it_patch = mlp.patches_[l].patch_;
      float* it_dx = mlp.patches_[l].dx_;
      float* it_dy = mlp.patches_[l].dy_;
      if(!doWarping){
        extractPatchFromImage(extractedPatch,pyr.imgs_[l],c_level,false);
      } else {
        extractWarpedPatchFromImage(extractedPatch,pyr.imgs_[l],c_level,aff,false);
      }
      float* it_patch_extracted = extractedPatch.patch_;
      for(int y=0; y<patch_size; ++y){
        for(int x=0; x<patch_size; ++x, ++it_patch, ++it_patch_extracted, ++it_dx, ++it_dy){
          const float res = *it_patch_extracted - *it_patch;
          const float Jx = -pow(0.5,l)*(*it_dx);
          const float Jy = -pow(0.5,l)*(*it_dy);
          mean_diff += res;
          b(count*patch_size*patch_size+y*patch_size+x,0) = res;
          if(!doWarping){
            mean_diff_dx += Jx;
            mean_diff_dy += Jy;
            A(count*patch_size*patch_size+y*patch_size+x,0) = Jx;
            A(count*patch_size*patch_size+y*patch_size+x,1) = Jy;
          } else {
            const float Jx_warp = Jx*affInv(0,0)+Jy*affInv(1,0);
            const float Jy_warp = Jx*affInv(0,1)+Jy*affInv(1,1);
            mean_diff_dx += Jx_warp;
            mean_diff_dy += Jy_warp;
            A(count*patch_size*patch_size+y*patch_size+x,0) = Jx_warp;
            A(count*patch_size*patch_size+y*patch_size+x,1) = Jy_warp;
          }
        }
      }
    }
  }
  mean_diff = mean_diff/static_cast<float>(numLevel*patch_size*patch_size);
  mean_diff_dx = mean_diff_dx/static_cast<float>(numLevel*patch_size*patch_size);
  mean_diff_dy = mean_diff_dy/static_cast<float>(numLevel*patch_size*patch_size);
  b.array() -= mean_diff;
  A.col(0).array() -= mean_diff_dx;
  A.col(1).array() -= mean_diff_dy;
  return true;
}
template<int n_levels,int patch_size>
bool getLinearAlignEquationsReduced(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr, const int l1, const int l2,
                                    const bool doWarping, Eigen::Matrix2f& A_red, Eigen::Vector2f& b_red){
  bool success = getLinearAlignEquations(mlp,pyr,l1,l2,doWarping,mlp.A_,mlp.b_);
  if(success){
    mlp.mColPivHouseholderQR_.compute(mlp.A_);
    b_red = (mlp.mColPivHouseholderQR_.householderQ().inverse()*mlp.b_).template block<2,1>(0,0);
    A_red = mlp.mColPivHouseholderQR_.matrixR().template block<2,2>(0,0);
    A_red(1,0) = 0.0;
    A_red = A_red*mlp.mColPivHouseholderQR_.colsPermutation();
  }
  return success;
}
template<int n_levels,int patch_size>
bool getLinearAlignEquationsReduced(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr, const int l1, const int l2,
                                    const bool doWarping, Eigen::Matrix2d& A_red, Eigen::Vector2d& b_red){
  Eigen::Matrix2f A_red_;
  Eigen::Vector2f b_red_;
  bool success = getLinearAlignEquationsReduced(mlp,pyr,l1,l2,doWarping,A_red_,b_red_);
  if(success){
    A_red = A_red_.cast<double>();
    b_red = b_red_.cast<double>();
  }
  return success;
}

template<int n_levels,int patch_size>
bool align2D_old(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr,
                 const int l1, const int l2, const bool doWarping, const int maxIter = 10, const double minPixUpd = 0.03){
  int numLevel = 0;
  for(int l = l1; l <= l2; l++){
    const cv::Point2f c_level = levelTranformCoordinates(mlp.get_c(),pyr,0,l);
    if(mlp.isValidPatch_[l] && isPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c_level)){
      numLevel++;
      mlp.patches_[l].computeGradientParameters();
    }
  }
  if(numLevel==0){
    return false;
  }
  Eigen::Matrix3f aff = Eigen::Matrix3f::Identity();
  if(doWarping){
    aff.block<2,2>(0,0) = mlp.get_affineTransform();
  }
  Eigen::Matrix3f affInv = aff.inverse();
  const int halfpatch_size = patch_size/2;
  bool converged=false;
  Eigen::Matrix3f H; H.setZero();
  for(int l = l1; l <= l2; l++){
    const cv::Point2f c_level = levelTranformCoordinates(mlp.get_c(),pyr,0,l);
    if(mlp.isValidPatch_[l] && isPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c_level)){
      mlp.patches_[l].computeGradientParameters();
      H(0,0) += pow(0.25,l)*mlp.patches_[l].H_(0,0);
      H(0,1) += pow(0.25,l)*mlp.patches_[l].H_(0,1);
      H(1,0) += pow(0.25,l)*mlp.patches_[l].H_(1,0);
      H(1,1) += pow(0.25,l)*mlp.patches_[l].H_(1,1);
      H(2,0) += pow(0.5,l)*mlp.patches_[l].H_(2,0);
      H(2,1) += pow(0.5,l)*mlp.patches_[l].H_(2,1);
      H(0,2) += pow(0.5,l)*mlp.patches_[l].H_(0,2);
      H(1,2) += pow(0.5,l)*mlp.patches_[l].H_(1,2);
      H(2,2) += mlp.patches_[l].H_(2,2);
    }
  }
  Eigen::Matrix3f Hinv = H.inverse();
  float mean_diff = 0;


  // termination condition
  const float min_update_squared = minPixUpd*minPixUpd;
  cv::Point2f c_temp = mlp.get_c();
  Eigen::Vector3f update; update.setZero();
  for(int iter = 0; iter<maxIter; ++iter){
    if(isnan(c_temp.x) || isnan(c_temp.y)){
      assert(false);
      return false;
    }
    Eigen::Vector3f Jres; Jres.setZero();
    int count = 0;
    for(int l = l1; l <= l2; l++){
      const cv::Point2f c_level = levelTranformCoordinates(c_temp,pyr,0,l);
      if(mlp.isValidPatch_[l] && isPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c_level)){
        const int refStep = pyr.imgs_[l].step.p[0];

        float* it_patch = mlp.patches_[l].patch_;
        float* it_dx = mlp.patches_[l].dx_;
        float* it_dy = mlp.patches_[l].dy_;
        if(!doWarping){
          const int u_r = floor(c_level.x);
          const int v_r = floor(c_level.y);
          if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= pyr.imgs_[l].cols-halfpatch_size || v_r >= pyr.imgs_[l].rows-halfpatch_size){ // TODO: check limits
            return false;
          }
          // compute interpolation weights
          const float subpix_x = c_level.x-u_r;
          const float subpix_y = c_level.y-v_r;
          const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
          const float wTR = subpix_x * (1.0-subpix_y);
          const float wBL = (1.0-subpix_x)*subpix_y;
          const float wBR = subpix_x * subpix_y;

          uint8_t* it_img;        // loop through search_patch, interpolate
          for(int y=0; y<patch_size; ++y){
            it_img = (uint8_t*) pyr.imgs_[l].data + (v_r+y-halfpatch_size)*refStep + u_r-halfpatch_size;
            for(int x=0; x<patch_size; ++x, ++it_img, ++it_patch, ++it_dx, ++it_dy){
              const float intensity = wTL*it_img[0] + wTR*it_img[1] + wBL*it_img[refStep] + wBR*it_img[refStep+1];
              const float res = intensity - *it_patch + mean_diff;
              Jres[0] -= pow(0.5,l)*res*(*it_dx);
              Jres[1] -= pow(0.5,l)*res*(*it_dy);
              Jres[2] -= res;
            }
          }
        } else {
          for(int y=0; y<patch_size; ++y){ // TODO: renaming
            for(int x=0; x<patch_size; ++x, ++it_patch, ++it_dx, ++it_dy){
              const float dx = x - halfpatch_size + 0.5;
              const float dy = y - halfpatch_size + 0.5;
              const float wdx = aff(0,0)*dx + aff(0,1)*dy;
              const float wdy = aff(1,0)*dx + aff(1,1)*dy;
              const float u_pixel = c_level.x + wdx - 0.5;
              const float v_pixel = c_level.y + wdy - 0.5;
              const int u_pixel_r = floor(u_pixel);
              const int v_pixel_r = floor(v_pixel);
              if(u_pixel_r < 0 || v_pixel_r < 0 || u_pixel_r >= pyr.imgs_[l].cols-1 || v_pixel_r >= pyr.imgs_[l].rows-1){ // TODO: check limits
                return false;
              }
              const float pixel_subpix_x = u_pixel-u_pixel_r;
              const float pixel_subpix_y = v_pixel-v_pixel_r;
              const float pixel_wTL = (1.0-pixel_subpix_x) * (1.0-pixel_subpix_y);
              const float pixel_wTR = pixel_subpix_x * (1.0-pixel_subpix_y);
              const float pixel_wBL = (1.0-pixel_subpix_x) * pixel_subpix_y;
              const float pixel_wBR = pixel_subpix_x * pixel_subpix_y;
              const uint8_t* pixel_data = (uint8_t*) pyr.imgs_[l].data + v_pixel_r*refStep + u_pixel_r;
              const float pixel_intensity = pixel_wTL*pixel_data[0] + pixel_wTR*pixel_data[1] + pixel_wBL*pixel_data[refStep] + pixel_wBR*pixel_data[refStep+1];
              const float res = pixel_intensity - *it_patch + mean_diff;
              Jres[0] -= pow(0.5,l)*res*(*it_dx);
              Jres[1] -= pow(0.5,l)*res*(*it_dy);
              Jres[2] -= res;
            }
          }
        }
        count++;
      }
    }
    if(count==0){
      return false;
    }
    if(!doWarping){
      update = Hinv * Jres;
    } else {
      update = aff * Hinv * Jres;
    }
    c_temp.x += update[0];
    c_temp.y += update[1];
    mean_diff += update[2];

    if(update[0]*update[0]+update[1]*update[1] < min_update_squared){
      converged=true;
      break;
    }
  }

  mlp.set_c(c_temp);
  return converged;
}

template<int n_levels,int patch_size>
bool align2D(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr,
             const int l1, const int l2, const bool doWarping, const int maxIter = 10, const double minPixUpd = 0.03){
  // termination condition
  const float min_update_squared = minPixUpd*minPixUpd;
  cv::Point2f c_backup = mlp.get_c();
  Eigen::Vector2f update; update.setZero();
  bool converged = false;
  for(int iter = 0; iter<maxIter; ++iter){
    if(isnan(mlp.get_c().x) || isnan(mlp.get_c().y)){
      assert(false);
      mlp.set_c(c_backup);
      return false;
    }
    if(!getLinearAlignEquations(mlp,pyr,l1,l2,doWarping,mlp.A_,mlp.b_)){
      mlp.set_c(c_backup);
      return false;
    }
    update = mlp.A_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(mlp.b_);
    const cv::Point2f c_new(mlp.get_c().x + update[0],mlp.get_c().y + update[1]);
    mlp.set_c(c_new);

    if(update[0]*update[0]+update[1]*update[1] < min_update_squared){
      converged=true;
      break;
    }
  }
  if(converged == false){
    mlp.set_c(c_backup);
  }
  return converged;
}

template<int n_levels,int patch_size>
bool align2DSingleLevel(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr, const int l, const bool doWarping){
  return align2D(mlp,pyr,l,l,doWarping);
}
template<int n_levels,int patch_size>
void align2DComposed(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr,const int start_level,const int end_level, const int num_seq, const bool doWarping){
  cv::Point2f c_backup = mlp.get_c();
  mlp.status_.matchingStatus_ = FOUND;
  for(int l = end_level+num_seq;l>=end_level;--l){
    if(!align2D(mlp,pyr,l,start_level,doWarping)){
      mlp.status_.matchingStatus_ = NOTFOUND;
      break;
    }
  }
  if(mlp.status_.matchingStatus_ == NOTFOUND){
    mlp.set_c(c_backup);
  }
}

template<int n_levels,int patch_size,int nMax>
class MultilevelPatchSet{
 public:
  MultilevelPatchFeature<n_levels,patch_size> features_[nMax];
  bool isValid_[nMax];
  int maxIdx_;
  MultilevelPatchSet(const Camera* mpCamera = nullptr){
    reset(mpCamera);
  }
  ~MultilevelPatchSet(){}
  void reset(const Camera* mpCamera = nullptr){
    maxIdx_ = 0;
    for(unsigned int i=0;i<nMax;i++){
      features_[i].setCamera(mpCamera);
      isValid_[i] = false;
    }
  }
  bool getFreeIndex(int& ind) const{
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == false){
        ind = i;
        return true;
      }
    }
    return false;
  }
  int getValidCount() const{
    int count = 0;
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == true) ++count;
    }
    return count;
  }
  int addFeature(const MultilevelPatchFeature<n_levels,patch_size>& feature){
    int newInd = -1;
    if(getFreeIndex(newInd)){
      features_[newInd] = feature;
      features_[newInd].idx_ = maxIdx_++;
      isValid_[newInd] = true;
    } else {
      std::cout << "Feature Manager: maximal number of feature reached" << std::endl;
    }
    return newInd;
  }
  float getAverageScore(){
    float averageScore = 0;
    int count = 0;
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == true){
        averageScore += std::max(features_[i].s_,0.0f);
        ++count;
      }
    }
    if(count>0) averageScore /= count;
    return averageScore;
  }
};

template<int n_levels,int patch_size,int nMax>
std::unordered_set<unsigned int> addBestCandidates(MultilevelPatchSet<n_levels,patch_size,nMax>& mlpSet, std::list<cv::Point2f>& candidates, const ImagePyramid<n_levels>& pyr, const double initTime,
                                                   const int l1, const int l2, const int maxN, const int nDetectionBuckets, const double scoreDetectionExponent,
                                                   const double penaltyDistance, const double zeroDistancePenalty, const bool requireMax, const float minScore){
  std::unordered_set<unsigned int> newSet;
  std::list<MultilevelPatchFeature<n_levels,patch_size>> candidatesWithPatch;
  float maxScore = -1.0;
  for(auto it = candidates.begin(); it != candidates.end(); ++it){
    candidatesWithPatch.emplace_back();
    candidatesWithPatch.rbegin()->reset(-1,initTime);
    candidatesWithPatch.rbegin()->set_c(*it);
    if(isMultilevelPatchInFrame(*candidatesWithPatch.rbegin(),pyr,n_levels-1,true)){
      extractMultilevelPatchFromImage(*candidatesWithPatch.rbegin(),pyr,n_levels-1,true);
      candidatesWithPatch.rbegin()->computeMultilevelShiTomasiScore(l1,l2);
      if(candidatesWithPatch.rbegin()->s_ > maxScore) maxScore = candidatesWithPatch.rbegin()->s_;
    } else {
      candidatesWithPatch.rbegin()->s_ = -1;
    }
  }
  if(maxScore <= minScore){
    return newSet;
  }


  // Make buckets and fill based on score
  std::vector<std::unordered_set<MultilevelPatchFeature<n_levels,patch_size>*>> buckets(nDetectionBuckets,std::unordered_set<MultilevelPatchFeature<n_levels,patch_size>*>());
  unsigned int newBucketID;
  float relScore;
  for (auto it_cand = candidatesWithPatch.begin(); it_cand != candidatesWithPatch.end(); ++it_cand) {
    relScore = (it_cand->s_-minScore)/(maxScore-minScore);
    if(relScore > 0.0){
      newBucketID = std::ceil((nDetectionBuckets-1)*(pow(relScore,static_cast<float>(scoreDetectionExponent))));
      if(newBucketID>nDetectionBuckets-1) newBucketID = nDetectionBuckets-1;
      buckets[newBucketID].insert(&(*it_cand));
    }
  }

  // Move buckets based on current features
  double d2;
  double t2 = pow(penaltyDistance,2);
  bool doDelete;
  MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
  for(unsigned int i=0;i<nMax;i++){
    if(mlpSet.isValid_[i]){
      mpFeature = &mlpSet.features_[i];
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
  }

  // Incrementally add features and update candidate buckets
  MultilevelPatchFeature<n_levels,patch_size>* mpNewFeature;
  int addedCount = 0;
  for (int bucketID = nDetectionBuckets-1;bucketID >= 0+static_cast<int>(!requireMax);bucketID--) {
    while(!buckets[bucketID].empty() && addedCount < maxN && mlpSet.getValidCount() != nMax) {
      mpNewFeature = *(buckets[bucketID].begin());
      buckets[bucketID].erase(mpNewFeature);
      const int ind = mlpSet.addFeature(*mpNewFeature);
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

template <int n_levels>
void detectFastCorners(const ImagePyramid<n_levels>& pyr, std::list<cv::Point2f>& candidates,int l,int detectionThreshold) {
  std::vector<cv::KeyPoint> keypoints;
  cv::FastFeatureDetector feature_detector_fast(detectionThreshold, true);
  feature_detector_fast.detect(pyr.imgs_[l], keypoints);
  cv::Point2f level_c;
  for (auto it = keypoints.cbegin(), end = keypoints.cend(); it != end; ++it) {
    level_c = cv::Point2f(it->pt.x, it->pt.y);
    candidates.push_back(levelTranformCoordinates(level_c,pyr,l,0));
  }
}

template<int n_levels,int patch_size,int nMax>
void pruneCandidates(const MultilevelPatchSet<n_levels,patch_size,nMax>& mlpSet, std::list<cv::Point2f>& candidates){ // TODO: add corner motion dependency
  constexpr float t2 = patch_size*patch_size; // TODO: param
  bool prune;
  const MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
  for (auto it = candidates.begin(); it != candidates.end();) {
    prune = false;
    for(unsigned int i=0;i<nMax;i++){
      if(mlpSet.isValid_[i]){
        mpFeature = &mlpSet.features_[i];
        if(pow(it->x-mpFeature->c_.x,2) + pow(it->y-mpFeature->c_.y,2) < t2){ // TODO: check inFrame
          prune = true;
          break;
        }
      }
    }
    if(prune){
      it = candidates.erase(it);
    } else {
      it++;
    }
  }
}

}


#endif /* ROVIO_COMMON_VISION_HPP_ */
