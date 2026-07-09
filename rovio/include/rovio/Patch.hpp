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

#ifndef ROVIO_PATCH_HPP_
#define ROVIO_PATCH_HPP_

#include "lightweight_filtering/common.hpp"
#include "rovio/FeatureCoordinates.hpp"

namespace rovio{

/** \brief %Patch with selectable patchSize.
 *
 *   @tparam patchSize - Edge length of the patch in pixels. Value must be a multiple of 2!
 */
template<int patchSize>
class Patch {
 public:
  float patch_[patchSize*patchSize] __attribute__ ((aligned (16)));  /**<Array, containing the intensity values of the patch.*/
  float patchWithBorder_[(patchSize+2)*(patchSize+2)] __attribute__ ((aligned (16)));  /**<Array, containing the intensity values of the expanded patch.
                                                                                 This expanded patch is necessary for the intensity gradient calculation.*/
  mutable float dx_[patchSize*patchSize] __attribute__ ((aligned (16)));  /**<Array, containing the intensity gradient component in x-direction for each patch pixel.*/
  mutable float dy_[patchSize*patchSize] __attribute__ ((aligned (16)));  /**<Array, containing the intensity gradient component in y-direction for each patch pixel.*/
  mutable Eigen::Matrix3f H_;  /**<Hessian matrix of the patch (necessary for the patch alignment).*/
  mutable float s_;  /**<Shi-Tomasi Score (smaller eigenvalue of H_).*/
  mutable float e0_;  /**<Smaller eigenvalue of H_.*/
  mutable float e1_;  /**<Larger eigenvalue of H_.*/
  mutable bool validGradientParameters_;  /**<True, if the gradient parameters (patch gradient components dx_ dy_, Hessian H_, Shi-Thomasi Score s_) have been computed.
                                  \see computeGradientParameters()*/
  /** \brief Constructor
   */
  Patch(){
    static_assert(patchSize%2==0,"Patch patchSize must be a multiple of 2");
    validGradientParameters_ = false;
    s_ = 0.0;
    e0_ = 0.0;
    e1_ = 0.0;
  }
  /** \brief Destructor
   */
  virtual ~Patch(){}

  /** \brief Computes the gradient parameters of the patch (patch gradient components dx_ dy_, Hessian H_, Shi-Tomasi Score s_, Eigenvalues of the Hessian e0_ and e1_).
   *         The expanded patch patchWithBorder_ must be set.
   *         Sets validGradientParameters_ afterwards to true.
   */
  void computeGradientParameters() const{
    if(!validGradientParameters_){
      H_.setZero();
      const int refStep = patchSize+2;
      float* it_dx = dx_;
      float* it_dy = dy_;
      const float* it;
      Eigen::Vector3f J;
      J.setZero();
      for(int y=0; y<patchSize; ++y){
        it = patchWithBorder_ + (y+1)*refStep + 1;
        for(int x=0; x<patchSize; ++x, ++it, ++it_dx, ++it_dy){
          J[0] = 0.5 * (it[1] - it[-1]);
          J[1] = 0.5 * (it[refStep] - it[-refStep]);
          J[2] = 1;
          *it_dx = J[0];
          *it_dy = J[1];
          H_ += J*J.transpose();
        }
      }
      const float dXX = H_(0,0)/(patchSize*patchSize);
      const float dYY = H_(1,1)/(patchSize*patchSize);
      const float dXY = H_(0,1)/(patchSize*patchSize);

      e0_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
      e1_ = 0.5 * (dXX + dYY + sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
      s_ = e0_+e1_;
      validGradientParameters_ = true;
    }
  }

  /** \brief Extracts and sets the patch intensity values (patch_) from the intensity values of the
   *         expanded patch (patchWithBorder_).
   */
  void extractPatchFromPatchWithBorder(){
    float* it_patch = patch_;
    float* it_patchWithBorder;
    for(int y=1; y<patchSize+1; ++y, it_patch += patchSize){
      it_patchWithBorder = patchWithBorder_ + y*(patchSize+2) + 1;
      for(int x=0; x<patchSize; ++x)
        it_patch[x] = it_patchWithBorder[x];
    }
  }

  /** \brief Returns the Shi-Tomasi Score s_.
   *
   *   Computes and sets the gradient parameters, which are the patch gradient components dx_ dy_, the Hessian H_ and the Shi-Tomasi Score s_.
   *   \see computeGradientParameters()
   *   @return the Shi-Tomasi Score s_ ( smaller eigenvalue of the Hessian H_ ).
   */
  float getScore() const{
    computeGradientParameters();
    return s_;
  }

  /** \brief Returns the Hessian Matrix H_.
   *
   *   Computes and sets the gradient parameters, which are the patch gradient components dx_ dy_, the Hessian H_ and the Shi-Tomasi Score s_.
   *   @return the Hessian Matrix H_ .
   */
  Eigen::Matrix3f getHessian() const{
    computeGradientParameters();
    return H_;
  }

  /** \brief Draws the patch into an image.
   *
   *   @param drawImg    - Image in which the patch should be drawn.
   *   @param c          - Pixel coordinates of the left upper patch corner.
   *   @param stretch    - %Patch drawing magnification factor.
   *   @param withBorder - Draw either the patch patch_ (withBorder = false) or the expanded patch
   *                       patchWithBorder_ (withBorder = true).
   */
  void drawPatch(cv::Mat& drawImg,const cv::Point2i& c,int stretch = 1,const bool withBorder = false) const{
    const int refStepY = drawImg.step.p[0];
    const int refStepX = drawImg.step.p[1];
    uint8_t* img_ptr;
    const float* it_patch;
    if(withBorder){
      it_patch = patchWithBorder_;
    } else {
      it_patch = patch_;
    }
    for(int y=0; y<patchSize+2*(int)withBorder; ++y, it_patch += patchSize+2*(int)withBorder){
      img_ptr = (uint8_t*) drawImg.data + (c.y+y*stretch)*refStepY + c.x*refStepX;
      for(int x=0; x<patchSize+2*(int)withBorder; ++x)
        for(int i=0;i<stretch;++i){
          for(int j=0;j<stretch;++j){
            img_ptr[x*stretch*refStepX+i*refStepY+j*refStepX+0] = (uint8_t)(it_patch[x]);
            if(drawImg.channels() == 3){
              img_ptr[x*stretch*refStepX+i*refStepY+j*refStepX+1] = (uint8_t)(it_patch[x]);
              img_ptr[x*stretch*refStepX+i*refStepY+j*refStepX+2] = (uint8_t)(it_patch[x]);
            }
          }
        }
    }
  }

  /** \brief Draws the patch borders into an image.
   *
   * @param drawImg     - Image in which the patch borders should be drawn.
   * @param c           - Coordinates of the patch in the reference image.
   * @param s           - Scaling factor.
   * @param color       - Line color.
   */
  void drawPatchBorder(cv::Mat& drawImg,const FeatureCoordinates& c,const float s, const cv::Scalar& color) const{
    const double half_length = s*patchSize/2;
    if(c.isInFront() && c.com_warp_c()){
      cv::Point2f c1 = c.get_patchCorner(half_length,half_length).get_c();
      cv::Point2f c2 = c.get_patchCorner(half_length,-half_length).get_c();
      cv::line(drawImg,c1,c2,color,1);
      c1 = c.get_patchCorner(-half_length,-half_length).get_c();
      cv::line(drawImg,c2,c1,color,1);
      c2 = c.get_patchCorner(-half_length,half_length).get_c();
      cv::line(drawImg,c1,c2,color,1);
      c1 = c.get_patchCorner(half_length,half_length).get_c();
      cv::line(drawImg,c2,c1,color,1);
    }
  }

  /** \brief Checks if a patch at a specific image location is still within the reference image.
   *
   *   @param img        - Reference Image.
   *   @param c          - Coordinates of the patch in the reference image.
   *   @param withBorder - Check, using either the patch-patchSize of Patch::patch_ (withBorder = false) or the patch-patchSize
   *                       of the expanded patch Patch::patchWithBorder_ (withBorder = true).
   *   @return true, if the patch is completely located within the reference image.
   */
  static bool isPatchInFrame(const cv::Mat& img,const FeatureCoordinates& c,const bool withBorder = false){
    if(c.isInFront() && c.com_warp_c()){
      const int halfpatch_size = patchSize/2+(int)withBorder;
      if(c.isNearIdentityWarping()){
        if(c.get_c().x < halfpatch_size || c.get_c().y < halfpatch_size || c.get_c().x > img.cols-halfpatch_size || c.get_c().y > img.rows-halfpatch_size){
          return false;
        } else {
          return true;
        }
      } else {
        for(int y=0; y<2*halfpatch_size; y += 2*halfpatch_size-1){
          for(int x=0; x<2*halfpatch_size; x += 2*halfpatch_size-1){
            const float dx = x - halfpatch_size + 0.5;
            const float dy = y - halfpatch_size + 0.5;
            const float wdx = c.get_warp_c()(0,0)*dx + c.get_warp_c()(0,1)*dy;
            const float wdy = c.get_warp_c()(1,0)*dx + c.get_warp_c()(1,1)*dy;
            const float c_x = c.get_c().x+wdx - 0.5;
            const float c_y = c.get_c().y+wdy - 0.5;
            const int u_r = floor(c_x);
            const int v_r = floor(c_y);
            if(u_r < 0 || v_r < 0 || u_r >= img.cols-1 || v_r >= img.rows-1){
              return false;
            }
          }
        }
        return true;
      }
    } else {
      return false;
    }
  }

  /** \brief Extracts a patch from an image.
   *
   *   @param img        - Reference Image.
   *   @param c          - Coordinates of the patch in the reference image (subpixel coordinates possible).
   *   @param withBorder - If false, the patch object is only initialized with the patch data of the general patch (Patch::patch_).
   *                       If true, the patch object is initialized with both, the patch data of the general patch (Patch::patch_)
   *                       and the patch data of the expanded patch (Patch::patchWithBorder_).
   */
  void extractPatchFromImage(const cv::Mat& img,const FeatureCoordinates& c,const bool withBorder = false){
    assert(isPatchInFrame(img,c,withBorder));
    const int halfpatch_size = patchSize/2+(int)withBorder;
    const int refStep = img.step.p[0];

    // Get Pointers
    uint8_t* img_ptr;
    float* patch_ptr;
    if(withBorder){
      patch_ptr = patchWithBorder_;
    } else {
      patch_ptr = patch_;
    }

    if(c.isNearIdentityWarping()){
      const int u_r = floor(c.get_c().x);
      const int v_r = floor(c.get_c().y);

      // compute interpolation weights
      const float subpix_x = c.get_c().x-u_r;
      const float subpix_y = c.get_c().y-v_r;
      const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
      const float wTR = subpix_x * (1.0-subpix_y);
      const float wBL = (1.0-subpix_x)*subpix_y;
      const float wBR = subpix_x * subpix_y;
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
    } else {
      for(int y=0; y<2*halfpatch_size; ++y){
        for(int x=0; x<2*halfpatch_size; ++x, ++patch_ptr){
          const float dx = x - halfpatch_size + 0.5;
          const float dy = y - halfpatch_size + 0.5;
          const float wdx = c.get_warp_c()(0,0)*dx + c.get_warp_c()(0,1)*dy;
          const float wdy = c.get_warp_c()(1,0)*dx + c.get_warp_c()(1,1)*dy;
          const float u_pixel = c.get_c().x+wdx - 0.5;
          const float v_pixel = c.get_c().y+wdy - 0.5;
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
    }
    if(withBorder){
      extractPatchFromPatchWithBorder();
    }
    validGradientParameters_ = false;
  }
};

}


#endif /* ROVIO_PATCH_HPP_ */
