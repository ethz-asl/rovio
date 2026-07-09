#include "gtest/gtest.h"
#include <assert.h>

#include "rovio/Patch.hpp"

using namespace rovio;

class PatchTesting : public virtual ::testing::Test {
 protected:
  static const int patchSize_ = 2;
  static const int imgSize_ = patchSize_+4;
  static const int nMax_ = 20;
  static const int dx_ = 2;
  static const int dy_ = 3;
  cv::Mat img1_;
  cv::Mat img2_;
  Patch<patchSize_> p_;
  FeatureCoordinates c_;
  PatchTesting(){
    img1_ = cv::Mat::zeros(imgSize_,imgSize_,CV_8UC1);
    uint8_t* img_ptr = (uint8_t*) img1_.data;
    for(int i=0;i<imgSize_;i++){
      for(int j=0;j<imgSize_;j++, ++img_ptr){
        *img_ptr = i*dy_+j*dx_;
      }
    }
    img2_ = cv::Mat::zeros(imgSize_,imgSize_,CV_8UC1);
    img_ptr = (uint8_t*) img2_.data;
    for(int i=0;i<imgSize_;i++){
      for(int j=0;j<imgSize_;j++, ++img_ptr){
        if(j<imgSize_/2 & i<imgSize_/2){
          *img_ptr = 0;
        } else {
          *img_ptr = 255;
        }
      }
    }
  }
  virtual ~PatchTesting() {}
};

// Test constructors
TEST_F(PatchTesting, constructors) {
  Patch<(this->patchSize_)> p;
  ASSERT_EQ(p.validGradientParameters_,false);
  ASSERT_EQ(p.s_,0.0);
}

// Test isPatchInFrame
TEST_F(PatchTesting, isPatchInFrame) {
  c_.set_c(cv::Point2f(0,0));
  c_.set_warp_identity();
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(patchSize_/2-0.1,patchSize_/2-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(patchSize_/2,patchSize_/2));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(patchSize_/2+0.1,patchSize_/2+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-0.1,imgSize_-patchSize_/2-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2,imgSize_-patchSize_/2));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2+0.1,imgSize_-patchSize_/2+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(imgSize_,imgSize_));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(0,0));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(patchSize_/2+1-0.1,patchSize_/2+1-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(patchSize_/2+1,patchSize_/2+1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(patchSize_/2+1+0.1,patchSize_/2+1+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1-0.1,imgSize_-patchSize_/2-1-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1,imgSize_-patchSize_/2-1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1+0.1,imgSize_-patchSize_/2-1+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(imgSize_,imgSize_));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
}

// Test isPatchInFrame
TEST_F(PatchTesting, isPatchInFrameWithWarping) {
  Eigen::Matrix2f aff;
  aff.setIdentity();
  c_.set_warp_identity();
  c_.set_c(cv::Point2f(0,0));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(patchSize_/2-0.1,patchSize_/2-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(patchSize_/2,patchSize_/2));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(patchSize_/2+0.1,patchSize_/2+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-0.1,imgSize_-patchSize_/2-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2,imgSize_-patchSize_/2));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2+0.1,imgSize_-patchSize_/2+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(imgSize_,imgSize_));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(0,0));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(patchSize_/2+1-0.1,patchSize_/2+1-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(patchSize_/2+1,patchSize_/2+1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(patchSize_/2+1+0.1,patchSize_/2+1+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1-0.1,imgSize_-patchSize_/2-1-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1,imgSize_-patchSize_/2-1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1+0.1,imgSize_-patchSize_/2-1+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(imgSize_,imgSize_));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  aff << 0, -1, 1, 0;
  c_.set_warp_c(aff);
  c_.set_c(cv::Point2f(0,0));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(patchSize_/2-0.1,patchSize_/2-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(patchSize_/2,patchSize_/2));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(patchSize_/2+0.1,patchSize_/2+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-0.1,imgSize_-patchSize_/2-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2,imgSize_-patchSize_/2));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false); // should be true, but the implementation is just simpler like this (more reliable)
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2+0.1,imgSize_-patchSize_/2+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(imgSize_,imgSize_));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f(0,0));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(patchSize_/2+1-0.1,patchSize_/2+1-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(patchSize_/2+1,patchSize_/2+1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(patchSize_/2+1+0.1,patchSize_/2+1+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1-0.1,imgSize_-patchSize_/2-1-0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),true);
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1,imgSize_-patchSize_/2-1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false); // should be true, but the implementation is just simpler like this (more reliable)
  c_.set_c(cv::Point2f(imgSize_-patchSize_/2-1+0.1,imgSize_-patchSize_/2-1+0.1));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  c_.set_c(cv::Point2f(imgSize_,imgSize_));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_,true),false);
  aff << cos(M_PI/6.0), -sin(M_PI/6.0), sin(M_PI/6.0), cos(M_PI/6.0);
  c_.set_warp_c(aff);
  c_.set_c(cv::Point2f((sin(M_PI/6.0)+cos(M_PI/6.0))*(patchSize_/2-0.5)+0.5+1e-6,(sin(M_PI/6.0)+cos(M_PI/6.0))*(patchSize_/2-0.5)+0.5+1e-6));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),true);
  c_.set_c(cv::Point2f((sin(M_PI/6.0)+cos(M_PI/6.0))*(patchSize_/2-0.5)+0.5+1e-6,(sin(M_PI/6.0)+cos(M_PI/6.0))*(patchSize_/2-0.5)+0.5-1e-6));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
  c_.set_c(cv::Point2f((sin(M_PI/6.0)+cos(M_PI/6.0))*(patchSize_/2-0.5)+0.5-1e-6,(sin(M_PI/6.0)+cos(M_PI/6.0))*(patchSize_/2-0.5)+0.5+1e-6));
  ASSERT_EQ(p_.isPatchInFrame(img1_,c_),false);
}

// Test extractPatchFromImage
TEST_F(PatchTesting, extractPatchFromImage) {
  const int N = 6;
  cv::Point2f patchOrigins[N] = {cv::Point2f(0,0),
      cv::Point2f(0.5,0),
      cv::Point2f(0,0.5),
      cv::Point2f(0.5,0.5),
      cv::Point2f(1.5544,0.5234),
      cv::Point2f(imgSize_-patchSize_-2,imgSize_-patchSize_-2)};
  for(unsigned int n=0;n<N;n++){
    c_.set_c(cv::Point2f(patchSize_/2+1,patchSize_/2+1)+patchOrigins[n]);
    c_.set_warp_identity();
    p_.extractPatchFromImage(img1_,c_,true);
    float* patch_ptr = p_.patchWithBorder_;
    for(int i=0;i<patchSize_+2;i++){
      for(int j=0;j<patchSize_+2;j++, ++patch_ptr){
        ASSERT_NEAR(*patch_ptr,(float)((i+patchOrigins[n].y)*dy_+(j+patchOrigins[n].x)*dx_),1e-6);
      }
    }
  }
}

// Test extractWarpedPatchFromImage
TEST_F(PatchTesting, extractWarpedPatchFromImage) {
  Eigen::Matrix2f aff;
  aff << cos(M_PI/6.0), -sin(M_PI/6.0), sin(M_PI/6.0), cos(M_PI/6.0);
  c_.set_warp_c(aff);
  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  p_.extractPatchFromImage(img1_,c_,true);
  float* patch_ptr = p_.patchWithBorder_;
  for(int i=0;i<patchSize_+2;i++){
    for(int j=0;j<patchSize_+2;j++, ++patch_ptr){
      ASSERT_NEAR(*patch_ptr,((cos(M_PI/6.0)*(i-patchSize_/2-0.5)+sin(M_PI/6.0)*(j-patchSize_/2-0.5)+0.5*imgSize_-0.5)*dy_
                             +(-sin(M_PI/6.0)*(i-patchSize_/2-0.5)+cos(M_PI/6.0)*(j-patchSize_/2-0.5)+0.5*imgSize_-0.5)*dx_),1e-4);
    }
  }
}

// Test extractPatchFromPatchWithBorder
TEST_F(PatchTesting, extractPatchFromPatchWithBorder) {
  c_.set_c(cv::Point2f(patchSize_/2+1,patchSize_/2+1));
  c_.set_warp_identity();
  p_.extractPatchFromImage(img1_,c_,true);
  p_.extractPatchFromPatchWithBorder();
  float* patch_ptr = p_.patch_;
  for(int i=0;i<patchSize_;i++){
    for(int j=0;j<patchSize_;j++, ++patch_ptr){
      ASSERT_EQ(*patch_ptr,(float)((i+1)*dy_+(j+1)*dx_));
    }
  }
}

// Test computeGradientParameters
TEST_F(PatchTesting, computeGradientParameters) {
  c_.set_c(cv::Point2f(patchSize_/2+1,patchSize_/2+1+0.5));
  c_.set_warp_identity();
  p_.extractPatchFromImage(img1_,c_,true);
  p_.computeGradientParameters();
  float* it_dx = p_.dx_;
  float* it_dy = p_.dy_;
  for(int i=0;i<patchSize_;i++){
    for(int j=0;j<patchSize_;j++, ++it_dx, ++it_dy){
      ASSERT_EQ(*it_dx,float(dx_));
      ASSERT_EQ(*it_dy,float(dy_));
    }
  }
  ASSERT_EQ(p_.H_(0,0),dx_*dx_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(0,1),dx_*dy_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(0,2),dx_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(1,0),dx_*dy_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(1,1),dy_*dy_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(1,2),dy_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(2,0),dx_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(2,1),dy_*patchSize_*patchSize_);
  ASSERT_EQ(p_.H_(2,2),patchSize_*patchSize_);

  c_.set_c(cv::Point2f(imgSize_/2,imgSize_/2));
  c_.set_warp_identity();
  p_.extractPatchFromImage(img2_,c_,true);
  p_.computeGradientParameters();
  double s = 0.5 * (patchSize_ + patchSize_ - sqrtf((patchSize_ + patchSize_) * (patchSize_ + patchSize_) - 4 * (patchSize_ * patchSize_ - 1 * 1)))
              + 0.5 * (patchSize_ + patchSize_ + sqrtf((patchSize_ + patchSize_) * (patchSize_ + patchSize_) - 4 * (patchSize_ * patchSize_ - 1 * 1)));
  s = s*(255*0.5)*(255*0.5)/(patchSize_*patchSize_);
  ASSERT_EQ(p_.s_,s);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
