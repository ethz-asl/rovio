#include "rovio/common_vision2.hpp"
#include "gtest/gtest.h"
#include <assert.h>

using namespace rovio;

class PatchTesting : public virtual ::testing::Test {
 protected:
  static const int nLevels_ = 4;
  static const int patchSize_ = 2;
  static const int imgSize_ = patchSize_+4;
  static const int nMax_ = 20;
  static const int dx_ = 2;
  static const int dy_ = 3;
  cv::Mat img_;
  Patch2<patchSize_> p_;
  cv::Point2f c_;
  PatchTesting(){
    img_ = cv::Mat::zeros(imgSize_,imgSize_,CV_8UC1); // TODO
    uint8_t* img_ptr = (uint8_t*) img_.data;
    for(int i=0;i<imgSize_;i++){
      for(int j=0;j<imgSize_;j++, ++img_ptr){
        *img_ptr = i*dy_+j*dx_;
      }
    }
  }
  virtual ~PatchTesting() {
  }
};

// Test constructors
TEST_F(PatchTesting, constructors) {
  Patch2<patchSize_> p;
  ASSERT_EQ(p.validGradientParameters,false);
  ASSERT_EQ(p.s_,0.0);
  ASSERT_EQ(p.size_,patchSize_);
}

// Test isPatchInFrame
TEST_F(PatchTesting, isPatchInFrame) {
  c_ = cv::Point2f(0,0);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),false);
  c_ = cv::Point2f(patchSize_/2-0.1,patchSize_/2-0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),false);
  c_ = cv::Point2f(patchSize_/2,patchSize_/2);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),true);
  c_ = cv::Point2f(patchSize_/2+0.1,patchSize_/2+0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),true);
  c_ = cv::Point2f(imgSize_-patchSize_/2-0.1,imgSize_-patchSize_/2-0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),true);
  c_ = cv::Point2f(imgSize_-patchSize_/2,imgSize_-patchSize_/2);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),true);
  c_ = cv::Point2f(imgSize_-patchSize_/2+0.1,imgSize_-patchSize_/2+0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),false);
  c_ = cv::Point2f(imgSize_,imgSize_);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_),false);
  c_ = cv::Point2f(0,0);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),false);
  c_ = cv::Point2f(patchSize_/2+1-0.1,patchSize_/2+1-0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),false);
  c_ = cv::Point2f(patchSize_/2+1,patchSize_/2+1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),true);
  c_ = cv::Point2f(patchSize_/2+1+0.1,patchSize_/2+1+0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),true);
  c_ = cv::Point2f(imgSize_-patchSize_/2-1-0.1,imgSize_-patchSize_/2-1-0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),true);
  c_ = cv::Point2f(imgSize_-patchSize_/2-1,imgSize_-patchSize_/2-1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),true);
  c_ = cv::Point2f(imgSize_-patchSize_/2-1+0.1,imgSize_-patchSize_/2-1+0.1);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),false);
  c_ = cv::Point2f(imgSize_,imgSize_);
  ASSERT_EQ(isPatchInFrame(p_,img_,c_,true),false);
}

// Test extractPatchWithBorderFromImage
TEST_F(PatchTesting, extractPatchWithBorderFromImage) {
  const int N = 6;
  cv::Point2f patchOrigins[N] = {cv::Point2f(0,0),
      cv::Point2f(0.5,0),
      cv::Point2f(0,0.5),
      cv::Point2f(0.5,0.5),
      cv::Point2f(1.5544,0.5234),
      cv::Point2f(imgSize_-patchSize_-2,imgSize_-patchSize_-2)};
  for(unsigned int n=0;n<N;n++){
    c_ = cv::Point2f(patchSize_/2+1,patchSize_/2+1)+patchOrigins[n];
    extractPatchWithBorderFromImage(p_,img_,c_);
    uint8_t* patch_ptr = p_.patchWithBorder_;
    for(int i=0;i<patchSize_+2;i++){
      for(int j=0;j<patchSize_+2;j++, ++patch_ptr){
        ASSERT_EQ(*patch_ptr,(uint8_t)((i+patchOrigins[n].y)*dy_+(j+patchOrigins[n].x)*dx_));
      }
    }
  }
}

// Test extractPatchFromPatchWithBorder
TEST_F(PatchTesting, extractPatchFromPatchWithBorder) {
  c_ = cv::Point2f(patchSize_/2+1,patchSize_/2+1);
  extractPatchWithBorderFromImage(p_,img_,c_);
  p_.extractPatchFromPatchWithBorder();
  uint8_t* patch_ptr = p_.patch_;
  for(int i=0;i<patchSize_;i++){
    for(int j=0;j<patchSize_;j++, ++patch_ptr){
      ASSERT_EQ(*patch_ptr,(uint8_t)((i+1)*dy_+(j+1)*dx_));
    }
  }
}

// Test computeGradientParameters
TEST_F(PatchTesting, computeGradientParameters) {
  c_ = cv::Point2f(patchSize_/2+1,patchSize_/2+1+0.5);
  extractPatchWithBorderFromImage(p_,img_,c_);
  p_.computeGradientParameters();
  float* it_dx = p_.dx_;
  float* it_dy = p_.dy_;
  for(int i=0;i<patchSize_;i++){
    for(int j=0;j<patchSize_;j++, ++it_dx, ++it_dy){
      ASSERT_EQ(*it_dx,float(dx_));
      ASSERT_EQ(*it_dy,float(dy_));
    }
  }
  ASSERT_EQ(p_.s_,0.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
