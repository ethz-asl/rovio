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

/** \brief Defines the matching status of a MultilevelPatchFeature.
 *
 *  MatchingStatus is set in the function align2DComposed().
 */
enum MatchingStatus{
  NOTMATCHED,
  NOTFOUND,
  FOUND
};

/** \brief Defines the tracking status of a MultilevelPatchFeature.
 */
enum TrackingStatus{
  NOTTRACKED,
  FAILED,
  TRACKED
};

/** \brief Holds the  matching status (MatchingStatus) and tracking status (TrackingStatus) of a MultilevelPatchFeature.
 */
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

/** \brief Halfsamples an image.
 *
 *   @param imgIn - Input image.
 *   @param imgOut - Output image (halfsampled).
 */
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

/** \brief Image pyramid with selectable number of levels.
 *
 *   @tparam n_levels - Number of pyramid levels.
 */
template<int n_levels>
class ImagePyramid{
 public:
  ImagePyramid(){};
  ~ImagePyramid(){};
  cv::Mat imgs_[n_levels]; /**<Array, containing the pyramid images.*/
  cv::Point2f centers_[n_levels]; /**<Array, containing the image center coordinates (in pixel), defined in an
                                      image centered coordinate system of the image at level 0.*/

  /** \brief Initializes the image pyramid from an input image (level 0).
   *
   *   @param img   - Input image (level 0).
   *   @param useCv - Set to true, if opencv (cv::pyrDown) should be used for the pyramid creation.
   */
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

  /** \brief Copies the image pyramid.
   */
  ImagePyramid<n_levels>& operator=(const ImagePyramid<n_levels> &rhs) {
    for(unsigned int i=0;i<n_levels;i++){
      rhs.imgs_[i].copyTo(imgs_[i]);
      centers_[i] = rhs.centers_[i];
    }
    return *this;
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Vectors, pointing from the feature image location to the midpoints of the patch edges (expressed in pixels).
 *
 *  \see BearingCorners
 */
struct PixelCorners{
  cv::Point2f corners_[2];
  cv::Point2f& operator[](unsigned int i){
    return corners_[i];
  };
  const cv::Point2f& operator[](unsigned int i) const{
    return corners_[i];
  };
};

/** \brief Difference (see boxminus for bearing vectors) between feature bearing vector and bearing vectors
 *  of the midpoints of the corresponding patch edges.
 *
 *  \see PixelCorners
 */
struct BearingCorners{
  Eigen::Vector2d corners_[2];
  Eigen::Vector2d& operator[](unsigned int i){
    return corners_[i];
  };
  const Eigen::Vector2d& operator[](unsigned int i) const{
    return corners_[i];
  };
};

/** \brief %Patch with selectable size.
 *
 *   @tparam size - Edge length of the patch in pixels. Value must be a multiple of 2!
 */
template<int size>
class Patch {
 public:
  float patch_[size*size] __attribute__ ((aligned (16)));  /**<Array, containing the intensity values of the patch.*/
  float patchWithBorder_[(size+2)*(size+2)] __attribute__ ((aligned (16)));  /**<Array, containing the intensity values of the expanded patch.
                                                                                 This expanded patch is necessary for the intensity gradient calculation.*/
  float dx_[size*size] __attribute__ ((aligned (16)));  /**<Array, containing the intensity gradient component in x-direction for each patch pixel.*/
  float dy_[size*size] __attribute__ ((aligned (16)));  /**<Array, containing the intensity gradient component in y-direction for each patch pixel.*/
  Eigen::Matrix3f H_;  /**<Hessian matrix of the patch (necessary for the patch alignment).*/
  float s_;  /**<Shi-Tomasi Score (smaller eigenvalue of H_).*/
  float e0_;  /**<Smaller eigenvalue of H_.*/
  float e1_;  /**<Larger eigenvalue of H_.*/
  bool validGradientParameters_;  /**<True, if the gradient parameters (patch gradient components dx_ dy_, Hessian H_, Shi-Thomasi Score s_) have been computed.
                                  \see computeGradientParameters()*/
  /** \brief Constructor
   */
  Patch(){
    static_assert(size%2==0,"Patch size must be a multiple of 2");
    validGradientParameters_ = false;
    s_ = 0.0;
    e0_ = 0.0;
    e1_ = 0.0;
  }
  /** \brief Destructor
   */
  ~Patch(){}

  /** \brief Computes the gradient parameters of the patch (patch gradient components dx_ dy_, Hessian H_, Shi-Tomasi Score s_, Eigenvalues of the Hessian e0_ and e1_).
   *         The expanded patch patchWithBorder_ must be set.
   *         Sets validGradientParameters_ afterwards to true.
   */
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

    e0_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    e1_ = 0.5 * (dXX + dYY + sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    s_ = e0_;
    validGradientParameters_ = true;
  }

  /** \brief Extracts and sets the patch intensity values (patch_) from the intensity values of the
   *         expanded patch (patchWithBorder_).
   */
  void extractPatchFromPatchWithBorder(){
    float* it_patch = patch_;
    float* it_patchWithBorder;
    for(int y=1; y<size+1; ++y, it_patch += size){
      it_patchWithBorder = patchWithBorder_ + y*(size+2) + 1;
      for(int x=0; x<size; ++x)
        it_patch[x] = it_patchWithBorder[x];
    }
  }

  /** \brief Returns the Shi-Tomasi Score s_.
   *
   *   Computes and sets the gradient parameters, which are the patch gradient components dx_ dy_, the Hessian H_ and the Shi-Tomasi Score s_.
   *   \see computeGradientParameters()
   *   @return the Shi-Tomasi Score s_ ( smaller eigenvalue of the Hessian H_ ).
   */
  float getScore(){
    if(!validGradientParameters_) computeGradientParameters();
    return s_;
  }

  /** \brief Returns the Hessian Matrix H_.
   *
   *   Computes and sets the gradient parameters, which are the patch gradient components dx_ dy_, the Hessian H_ and the Shi-Tomasi Score s_.
   *   @return the Hessian Matrix H_ .
   */
  Eigen::Matrix3f getHessian(){
    if(!validGradientParameters_) computeGradientParameters();
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
    const int refStep = drawImg.step.p[0];
    uint8_t* img_ptr;
    const float* it_patch;
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

/** \brief Checks if an (aligned) patch at a specific image location is still within the reference image.
 *
 *   Note: The patch is assumed to be aligned with the image edges (patch is not warped).
 *   \see Patch
 *   \see isWarpedPatchInFrame()
 *   @tparam size      - Edge length of the patch in pixels (must be a multiple of 2).
 *   @param img        - Reference Image.
 *   @param c          - Center pixel coordinates of the patch.
 *   @param withBorder - Check, using either the patch-size of Patch::patch_ (withBorder = false) or the patch-size
 *                       of the expanded Patch::patchWithBorder_ (withBorder = true).
 *   @return true, if the patch is completely located within the image.
 */
template<int size>
bool isPatchInFrame(const Patch<size>& patch,const cv::Mat& img,const cv::Point2f& c,const bool withBorder = false){
  const int halfpatch_size = size/2+(int)withBorder;
  if(c.x < halfpatch_size || c.y < halfpatch_size || c.x > img.cols-halfpatch_size || c.y > img.rows-halfpatch_size){
    return false;
  } else {
    return true;
  }
}

/** \brief Checks if a (warped) patch at a specific image location is still within the reference image.
 *
 *   Note: The warped patch is most likely not aligned with the image edges.
 *   \see Patch
 *   \see isPatchInFrame()
 *   @tparam size      - Edge length of the patch in pixels (must be a multiple of 2).
 *   @param img        - Reference Image.
 *   @param c          - Center pixel coordinates of the patch in the reference image.
 *   @param aff        - Affine warping matrix (from the warped patch into the current image).
 *   @param withBorder - Check, using either the patch-size of Patch::patch_ (withBorder = false) or the patch-size
 *                       of the expanded patch Patch::patchWithBorder_ (withBorder = true).
 *   @return true, if the warped patch is completely located within the reference image.
 */
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

/** \brief Extracts an (aligned) patch from an image.
 *
 *   Note: The patch will be aligned with the image edges (patch not warped).
 *   \see Patch
 *   @tparam size      - Edge length of the patch in pixels (must be a multiple of 2).
 *   @param patch      - %Patch object, which will hold the patch data.
 *   @param img        - Reference Image.
 *   @param c          - Center pixel coordinates of the patch in the reference image (subpixel coordinates possible).
 *   @param withBorder - If false, the patch object is only initialized with the patch data of the general patch (Patch::patch_).
 *                       If true, the patch object is initialized with both, the patch data of the general patch (Patch::patch_)
 *                       and the patch data of the expanded patch (Patch::patchWithBorder_).
 */
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

/** \brief Extracts a (warped) patch from an image.
 *
 *   Note: The patch will most likely not be aligned with the image edges (patch is warped).
 *   \see Patch
 *   @tparam size      - Edge length of the patch in pixels (must be a multiple of 2).
 *   @param patch      - %Patch object, which will hold the patch data.
 *   @param img        - Reference Image.
 *   @param c          - Center pixel coordinates of the warped patch in the reference image (subpixel coordinates possible).
 *   @param aff        - Affine warping matrix (from the warped patch into the current image).
 *   @param withBorder - If false, the patch object is only initialized with the patch data of the general patch (Patch::patch_).
 *                       If true, the patch object is initialized with both, the patch data of the general patch (Patch::patch_)
 *                       and the patch data of the expanded patch (Patch::patchWithBorder_).
 */
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, handling feature coordinates.
 */
class FeatureCoordinates{
 public:
  int camID_; /**<%Camera ID.*/
  mutable cv::Point2f c_;  /**<Pixel coordinates of the feature.*/
  mutable bool valid_c_;  /**<Bool, indicating if the current feature pixel coordinates \ref c_ are valid.*/
  mutable LWF::NormalVectorElement nor_;  /**<Bearing vector, belonging to the feature.*/
  mutable bool valid_nor_;  /**<Bool, indicating if the current bearing vector \ref nor_ is valid.*/
  const Camera* mpCameras_;  /**<Pointer to the associated camera object.*/
  double sigma1_;  /**<Standard deviation in the direction of the major axis of the uncertainty ellipse belonging to \ref c_.*/
  double sigma2_;  /**<Standard deviation in the direction of the semi-major axis of the uncertainty ellipse belonging to \ref c_.*/
  double sigmaAngle_; /**<Angle between the x-axis and the major axis of the uncertainty ellipse belonging to \ref c_.*/
  Eigen::EigenSolver<Eigen::Matrix2d> es_;
  int* mpCamID_;  /**<Pointer to filter state variable.*/
  LWF::NormalVectorElement* mpNor_;  /**<Pointer to filter state variable.*/
  double depth_;  /**<Estimated feature depth (along the bearing vector).*/

  /** \brief Constructor
   */
  FeatureCoordinates(){
    mpCameras_ = nullptr;
    mpCamID_ = nullptr;
    mpNor_ = nullptr;
    resetCoordinates();
    camID_ = 0;
  }

  /** \brief Constructor
   */
  FeatureCoordinates(const Camera* mpCameras): mpCameras_(mpCameras){
    resetCoordinates();
    camID_ = 0;
  }

  /** \brief Resets the feature coordinates \ref c_, \ref nor_, \ref sigma1_, \ref sigma2_ and \ref sigmaAngle_.
   *
   *  Note, that the values of the feature coordinates \ref c_ and \ref nor_ are not deleted. They are just set invalid.
   */
  void resetCoordinates(){
    valid_c_ = false;
    valid_nor_ = false;
    sigma1_ = 0.0;
    sigma2_ = 0.0;
    sigmaAngle_ = 0.0;
  }

  /** \brief Links the feature with the filter state.
   *
   *   @param mpCamID - Pointer to camID in filter state.
   *   @param mpNor   - Pointer to bearing vector in filter state.
   */
  void linkToState(int* mpCamID, LWF::NormalVectorElement* mpNor){
    mpCamID_ = mpCamID;
    mpNor_ = mpNor;
  }

  /** \brief Sets the feature depth \ref depth_.
   *
   *   @param depth - Depth along the bearing vector \ref nor_.
   */
  void setDepth(const double& depth){
    depth_ = depth;
  }

  /** \brief Write the linked variable into the filter state.
   */
  void toState(){
    *mpCamID_ = camID_;
    *mpNor_ = get_nor();
  }

  /** \brief Reads the linked variable from the filter state.
   */
  void fromState(){
    camID_ = *mpCamID_;
    set_nor(*mpNor_);
  }

  /** \brief Get the feature's pixel coordinates \ref c_.
   *
   *   If there are no valid feature pixel coordinates \ref c_ available, the feature pixel coordinates are computed
   *   from the bearing vector \ref nor_ (if valid).
   *  @return the valid feature pixel coordinates \ref c_.
   */
  const cv::Point2f& get_c() const{
    if(!valid_c_){
      if(valid_nor_ && mpCameras_[camID_].bearingToPixel(nor_,c_)){
        valid_c_ = true;
      } else {
        std::cout << "ERROR: No valid coordinate data!" << std::endl;
      }
    }
    return c_;
  }

  /** \brief Get the feature's bearing vector \ref nor_.
   *
   *  If there is no valid bearing vector \ref nor_ available, the bearing vector is computed from the
   *  feature pixel coordinates \ref c_ (if valid).
   *  @return the valid bearing vector \ref nor_.
   */
  const LWF::NormalVectorElement& get_nor() const{
    if(!valid_nor_){
      if(valid_c_ && mpCameras_[camID_].pixelToBearing(c_,nor_)){
        valid_nor_ = true;
      } else {
        std::cout << "ERROR: No valid coordinate data!" << std::endl;
      }
    }
    return nor_;
  }

  /** \brief Get an appropriate bearing vector \ref nor_ located in an other camera frame, pointing to the same landmark.
   *
   *  @param otherCamID - Other camera ID.
   *  @return the appropriate bearing vector \ref nor_ of the camera frame with ID \ref otherCamID.
   */
  LWF::NormalVectorElement get_nor_other(const int otherCamID) const{
    if(camID_ != otherCamID){
      const QPD qDC = mpCameras_[otherCamID].qCB_*mpCameras_[camID_].qCB_.inverted(); // TODO: avoid double computation
      const V3D CrCD = mpCameras_[camID_].qCB_.rotate(V3D(mpCameras_[otherCamID].BrBC_-mpCameras_[camID_].BrBC_));
      const V3D CrCP = depth_*get_nor().getVec();
      const V3D DrDP = qDC.rotate(V3D(CrCP-CrCD));
      return LWF::NormalVectorElement(DrDP);
    } else {
      return get_nor();
    }
  }

  /** \brief Sets the feature pixel coordinates \ref c_ and declares them valid (\ref valid_c_ = true).
   *
   *  Note: The bearing vector nor_ is set invalid (valid_nor_ = false).
   *  \see set_nor()
   *  @param c - Feature pixel coordinates.
   */
  void set_c(const cv::Point2f& c){
    c_ = c;
    valid_c_ = true;
    valid_nor_ = false;
  }

  /** \brief Sets the feature's bearing vector \ref nor_ and declares it valid (\ref valid_nor_ = true).
   *
   *  Note: The feature pixel coordinates are set invalid (\ref valid_c_ = false).
   *  \see set_c()
   *  @param nor - Bearing vector.
   */
  void set_nor(const LWF::NormalVectorElement& nor){
    nor_ = nor;
    valid_nor_ = true;
    valid_c_ = false;
  }

  /** \brief Checks if the feature coordinates can be associated with a landmark in front of the camera.
   *
   *   @return true, if the feature coordinates can be associated with a landmark in front of the camera.
   */
  bool isInFront() const{
    return valid_c_ || (valid_nor_ && nor_.getVec()[2] > 0);
  }

  /** \brief Sets the feature coordinates standard deviation values \ref sigma1_, \ref sigma2_ and \ref sigmaAngle_
   *         from a 2x2 covariance matrix.
   *
   *  @param cov - Covariance matrix (2x2).
   */
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

/** \brief Draws a point at given feature coordinates.
 *
 *  @param drawImg - Image in which the point should be drawn.
 *  @param C       - Feature coordinates object. See class FeatureCoordinates.
 *  @param color   - Color of the point.
 */
static void drawPoint(cv::Mat& drawImg, FeatureCoordinates& C, const cv::Scalar& color){
  cv::Size size(2,2);
  cv::ellipse(drawImg,C.get_c(),size,0,0,360,color,-1,8,0);
}

/** \brief Draws an uncertainty ellipse at given feature coordinates.
 *
 *  @param drawImg         - Image in which the uncertainty ellipse should be drawn.
 *  @param C               - Feature coordinates object. Contains inter alia the uncertainty data. See class FeatureCoordinates.
 *  @param color           - Color of the ellipse.
 *  @param scaleFactor     - Scale Factor of the uncertainty ellipse. If set to 1, the ellipse axes lengths match the true
 *                           standard deviation values.
 *  @param withCenterPoint - Set to true if the center point of the ellipse should be drawn.
 */
static void drawEllipse(cv::Mat& drawImg, FeatureCoordinates& C, const cv::Scalar& color, double scaleFactor = 2.0, const bool withCenterPoint = true){
  if(withCenterPoint) drawPoint(drawImg,C,color);
  cv::ellipse(drawImg,C.get_c(),cv::Size(std::max(static_cast<int>(scaleFactor*C.sigma1_+0.5),1),std::max(static_cast<int>(scaleFactor*C.sigma2_+0.5),1)),C.sigmaAngle_*180/M_PI,0,360,color,1,8,0);
}

/** \brief Draws a line between two features.
 *
 *  @param drawImg     - Image in which the line should be drawn.
 *  @param C1          - Feature coordinates object 1. See class FeatureCoordinates.
 *  @param C2          - Feature coordinates object 2. See class FeatureCoordinates.
 *  @param color       - Color of the line.
 *  @param thickness   - Thickness of the line.
 */
static void drawLine(cv::Mat& drawImg, FeatureCoordinates& C1, FeatureCoordinates& C2, const cv::Scalar& color, int thickness = 2){
  cv::line(drawImg,C1.get_c(),C2.get_c(),color,thickness);
}

/** \brief Draws a text at specific feature coordinates.
 *
 *  @param drawImg     - Image in which the text should be drawn.
 *  @param C           - Feature coordinates object. Bottom-left corner of the text string in the image. See class FeatureCoordinates.
 *  @param s           - Text.
 *  @param color       - Color of the text.
 */
static void drawText(cv::Mat& drawImg, FeatureCoordinates& C, const std::string& s, const cv::Scalar& color){
  cv::putText(drawImg,s,C.get_c(),cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief A multilevel patch feature class.
 *
 *    @tparam n_levels   - Number of pyramid levels on which the feature is defined.
 *    @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!.
 *                         Note: The patches edge length (in pixels) is the same for each patch, independently from the pyramid level.
 */
template<int n_levels,int patch_size>
class MultilevelPatchFeature: public FeatureCoordinates{
 public:
  static const int nLevels_ = n_levels;  /**<Number of pyramid levels on which the feature is defined.*/
  Patch<patch_size> patches_[nLevels_];  /**<Array, holding the patches on each pyramid level.*/
  bool isValidPatch_[nLevels_];  /**<Array, specifying if there is a valid patch stored at the corresponding location in \ref patches_.*/
  static constexpr float warpDistance_ = static_cast<float>(patch_size); /**<Edge length of the patch in pixels.*/
  mutable PixelCorners pixelCorners_;  /**<Vectors, pointing from the feature coordinates to the midpoint of the patch edges.*/
  mutable bool valid_pixelCorners_;  /**<Specifies if the current pixel corners \ref pixelCorners_ are valid.*/
  mutable BearingCorners bearingCorners_;  /**<Vectors, pointing from the feature's bearing vector to the bearing vectors
                                               belonging to the pixel coordinates of the midpoint of the patch edges.*/
  mutable bool valid_bearingCorners_;  /**<Specifies if the current bearing corners \ref bearingCorners_ are valid.*/
  mutable Eigen::Matrix2f affineTransform_;  /**<Affine transformation matrix from current patch to the current frame. */
  mutable bool valid_affineTransform_;  /**<Specifies if the affine transformation \ref affineTransform_ is valid.*/
  int idx_;  /**<Feature ID.*/
  double initTime_;  /**<Time of feature initialization.*/
  double currentTime_;  /**<Time of last featutre measurement.*/
  Eigen::Matrix3f H_;  /**<Hessian matrix, corresponding to the multilevel patches.*/
  float e0_;  /**<Smaller eigenvalue of H_.*/
  float e1_;  /**<Larger eigenvalue of H_.*/
  float s_;  /**<Shi-Tomasi score of the multilevel patch feature. @todo define and store method of computation, add mutable */
  int totCount_;  /**<Number of images which have passed since feature initialization.*/
  Eigen::MatrixXf A_;  /**<A matrix of the linear system of equations, needed for the multilevel patch alignment.*/
  Eigen::MatrixXf b_;  /**<b matrix/vector of the linear system of equations, needed for the multilevel patch alignment.*/
  Eigen::ColPivHouseholderQR<Eigen::MatrixXf> mColPivHouseholderQR_;

  FeatureCoordinates log_previous_;
  FeatureCoordinates log_prediction_;
  FeatureCoordinates log_predictionC0_;
  FeatureCoordinates log_predictionC1_;
  FeatureCoordinates log_predictionC2_;
  FeatureCoordinates log_predictionC3_;
  FeatureCoordinates log_meas_;
  FeatureCoordinates log_current_;

  Status status_;  /**<MultilevelPatchFeature tracking and mapping status.*/
  std::map<MatchingStatus,int> cumulativeMatchingStatus_;  /**< Count for specific matching status.*/
  std::map<TrackingStatus,int> cumulativeTrackingStatus_;  /**< Count for specific tracking status.*/
  int inFrameCount_;  /**<How many times was the feature visible in a frame.*/
  std::map<double,Status> statistics_;  /**< Accumulation of status (index is the time)*/

  BearingCorners* mpBearingCorners_;  /**<Pointer to variable of filter state*/

  /** Constructor
   */
  MultilevelPatchFeature(){
    reset();
  }

  /** Constructor
   */
  MultilevelPatchFeature(const Camera* mpCameras): FeatureCoordinates(mpCameras){
    reset();
  }

  /** Destructor
   */
  ~MultilevelPatchFeature(){}

  /** \brief Links the MultilevelPatchFeature with a filter state.
   *
   * @param mpCamID            - Pointer to camera ID in filter state.
   * @param mpNor              - Pointer to bearing vector in filter state.
   * @param mpBearingCorners   - Pointer to bearing corners in filter state.
   */
  void linkToState(int* mpCamID, LWF::NormalVectorElement* mpNor, BearingCorners* mpBearingCorners){
    mpBearingCorners_ = mpBearingCorners;
    FeatureCoordinates::linkToState(mpCamID,mpNor);
  }

  /** \brief Write the linked variable into the filter state.
   */
  void toState(){
    FeatureCoordinates::toState();
    *mpBearingCorners_ = get_bearingCorners();
  }

  /** \brief Reads the linked variable from the filter state.
   */
  void fromState(){
    FeatureCoordinates::fromState();
    set_bearingCorners(*mpBearingCorners_);
  }

  /** \brief Sets the camera.

   * Sets the pointer to the camera array
   */
  void setCamera(const Camera* mpCameras){
    mpCameras_ = mpCameras;
  }

  /** \brief Resets the MultilevelPatchFeature.
   *
   * @param idx - feature ID
   * @initTime  - Time at initialization.
   */
  void reset(const int idx = -1, const double initTime = 0.0){
    resetCoordinates();
    idx_ = idx;
    initTime_ = initTime;
    currentTime_ = initTime;
    valid_pixelCorners_ = false;
    valid_bearingCorners_ = false;
    valid_affineTransform_ = false;
    H_.setIdentity();
    e0_ = 0;
    e1_ = 0;
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

  /** \brief Get the PixelCorners of the MultilevelPatchFeature.
   *
   * @return the valid PixelCorners.
   */
  const PixelCorners& get_pixelCorners() const{
    if(!valid_pixelCorners_){
      cv::Point2f tempPixel;
      LWF::NormalVectorElement tempNormal;
      if(valid_bearingCorners_){
        for(unsigned int i=0;i<2;i++){
          get_nor().boxPlus(bearingCorners_[i],tempNormal);
          if(!mpCameras_[camID_].bearingToPixel(tempNormal,tempPixel)){
            std::cout << "ERROR: Problem during bearing corner to pixel mapping!" << std::endl;
          }
          pixelCorners_[i] = tempPixel - get_c();
        }
        valid_pixelCorners_ = true;
      } else if(valid_affineTransform_) {
        for(unsigned int i=0;i<2;i++){
          pixelCorners_[i].x = affineTransform_(0,i)*warpDistance_;  // Parallelism is preserved in an affine transformation!
          pixelCorners_[i].y = affineTransform_(1,i)*warpDistance_;  // Parallelism is preserved in an affine transformation!
        }
        valid_pixelCorners_ = true;
      } else {
        std::cout << "ERROR: No valid corner data!" << std::endl;
      }
    }
    return pixelCorners_;
  }

  /** \brief Get the BearingCorners of the MultilevelPatchFeature.
   *
   * @return the valid BearingCorners.
   */
  const BearingCorners& get_bearingCorners() const{
    if(!valid_bearingCorners_){
      cv::Point2f tempPixel;
      LWF::NormalVectorElement tempNormal;
      get_pixelCorners();
      for(unsigned int i=0;i<2;i++){
        tempPixel = get_c()+pixelCorners_[i];
        if(!mpCameras_[camID_].pixelToBearing(tempPixel,tempNormal)){
          std::cout << "ERROR: Problem during pixel corner to bearing mapping!" << std::endl;
        }
        tempNormal.boxMinus(get_nor(),bearingCorners_[i]);
      }
      valid_bearingCorners_ = true;
    }
    return bearingCorners_;
  }

  /** \brief Get the affine transformation of the current patch, if not available computes it from the
   *  current pixelCorners or bearingCorners.
   *
   * @return the affine transformation matrix \ref affineTransform_.
   */
  const Eigen::Matrix2f& get_affineTransform() const{
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

  /** \brief Set the \ref pixelCorners_ of the MultilevelPatchFeature.
   *
   * Note: The validity of the bearingCorners_ and of the affineTransform_ is set to invalid
   *       (\ref valid_bearingCorners_ = false; \ref valid_affineTransform_ = false).
   * \see get_affineTransform()
   * @param pixelCorners
   */
  void set_pixelCorners(const PixelCorners& pixelCorners){
    pixelCorners_ = pixelCorners;
    valid_pixelCorners_ = true;
    valid_bearingCorners_ = false;
    valid_affineTransform_ = false;
  }

  /** \brief Set the \ref bearingCorners_ of the MultilevelPatchFeature.
   *
   * Note: The validity of the pixelCorners_ and of the affineTransform_ is set to invalid
   *       (\ref valid_pixelCorners_ = false; \ref valid_affineTransform_ = false).
   * @param bearingCorners
   */
  void set_bearingCorners(const BearingCorners& bearingCorners){
    bearingCorners_ = bearingCorners;
    valid_bearingCorners_ = true;
    valid_pixelCorners_ = false;
    valid_affineTransform_ = false;
  }

  /** \brief Set the affine transformation \ref affineTransform_ of the MultilevelPatchFeature.
   *
   * Note: The validity of the pixelCorners_ and of the bearingCorners_ is set to invalid
   *       (\ref valid_pixelCorners_ = false; \ref bearingCorners_ = false).
   * @param affineTransform - Affine transformation matrix between patch and current image.
   */
  void set_affineTransfrom(const Eigen::Matrix2f& affineTransform){
    affineTransform_ = affineTransform;
    valid_affineTransform_ = true;
    valid_bearingCorners_ = false;
    valid_pixelCorners_ = false;
  }

  /** \brief Increases the MultilevelPatchFeature statistics and resets the \ref status_.
   *
   * @param currentTime - Current time.
   */
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

  /** \brief How many times did a specific \ref MatchingStatus occur (current status included)?
   *
   * @param s - \ref MatchingStatus of interest.
   * @return the number of how many times the \ref MatchingStatus s occured.
   */
  int countMatchingStatistics(const MatchingStatus s) const{
    return cumulativeMatchingStatus_.at(s) + (int)(status_.matchingStatus_ == s);
  }

  /** \brief How many times did a specific \ref TrackingStatus occur (current status included)?
   *
   * @param s - \ref TrackingStatus of interest.
   * @return the number of how many times the \ref TrackingStatus s occured.
   */
  int countTrackingStatistics(const TrackingStatus s) const{
    return cumulativeTrackingStatus_.at(s) + (int)(status_.trackingStatus_ == s);
  }

  /** \brief How many times did a specific \ref MatchingStatus in the last n frames occur (current status included)?
   *
   * @param s - \ref MatchingStatus of interest.
   * @param n - \ref Last n frames.
   * @return the number of how many times the \ref MatchingStatus hass occured in the last n frames (current status included).
   */
  int countMatchingStatistics(const MatchingStatus s, const int n) const{
    int c = 0;
    auto it = statistics_.rbegin();
    for(int i=0;i<n-1 && it != statistics_.rend();++i){
      if(it->second.matchingStatus_ == s) c++;
      ++it;
    }
    return c + (int)(status_.matchingStatus_ == s);
  }

  /** \brief How many times did a specific \ref TrackingStatus in the last n frames occur (current status included)?
   *
   * @param s - \ref TrackingStatus of interest.
   * @param n - \ref Last n frames.
   * @return the number of how many times the \ref TrackingStatus has occured in the last n frames (current status included).
   */
  int countTrackingStatistics(const TrackingStatus s, const int n) const{
    int c = 0;
    auto it = statistics_.rbegin();
    for(int i=0;i<n-1 && it != statistics_.rend();++i){
      if(it->second.trackingStatus_ == s) c++;
      ++it;
    }
    return c + (int)(status_.trackingStatus_ == s);
  }

  /** \brief Returns the total count of frames since feature initialization (including the current).
   *
   * @return Total count of frames
   */
  int countTot() const{
    return totCount_+1;
  }

  /** \brief Returns the count of frames since feature initialization where the feature was in the frame (including the current).
   *
   * @return In frame count
   */
  int countTotInFrame() const{
    return inFrameCount_+(int)(status_.inFrame_);
  }

  /** \brief Get the local quality of the MultilevelPatchFeature.
   *         How is the tracking within the local range, OUTLIER/NOTFOUND is worse than not in image.
   *
   * @param localRange - Have only a look at the last "localRange" frames, where the feature was visible in the frame.
   * @return quality value in the range [0, 1] (ratio between tracking number and visibility number).
   *         1 means very good tracking quality.
   */
  double getLocalQuality(const int localRange = 10) const{
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

  /** \brief Get the local visibility quality of the MultilevelPatchFeature.
   *
   * @param localRange - Have only a look at the last "localRange" frames.
   * @return quality value in the range [0, 1] (ratio between visibility number and localRange).
   *         1 means that the feature was always visible in the considered frames.
   */
  double getLocalVisibilityQuality(const int localRange = 200) const{
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

  /** \brief How was the overall tracking of the feature. What is the tracking ratio of the feature, whereby the maximal value
   * is only obtained if a minimum of "frameCountRef" frames has passed since initialization (punishes feature with low feature
   * count)
   *
   * @param frameCountRef - Minimum of frames for maximal quality
   * @return quality value in the range [0, 1] (tracking ratio of feature).
   *         1 means that the feature was always tracked
   *         gets further penalized if a feature ha not a minimum of "frameCountRef" frames
   */
  double getGlobalQuality(const int frameCountRef = 100) const{
    const double trackingRatio = static_cast<double>(countTrackingStatistics(TRACKED))/static_cast<double>(countTot());
    return trackingRatio*std::min(static_cast<double>(countTot())/frameCountRef,1.0);
  }


  /** \brief Is the current feature a good feature. Combines different quality criteria for deciding if it is a good feature.
   * The product of local quality and visibility quality is compared with a threshold. This threshold depends on the global
   * quality (lower if the global quality is good).
   *
   * @param localRange           local range for local quality
   * @param localVisibilityRange local range for visibility quality
   * @param upper                if the global quality is bad (0) than the combination of local and visibility quality must be above this
   * @param lower                if the global quality is very good (1) than the combination of local and visibility quality must be above this
   * @return
   */
  bool isGoodFeature(const int localRange = 10, const int localVisibilityRange = 100, const double upper = 0.8, const double lower = 0.1) const{
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

  /** \brief Computes and sets the multilevel Shi-Tomasi Score \ref s_, considering a defined pyramid level interval.
   *
   * @param l1 - Start level (l1<l2)
   * @param l2 - End level (l1<l2)
   */
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
      e0_ = 0.5 * (dXX + dYY - sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
      e1_ = 0.5 * (dXX + dYY + sqrtf((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
      s_ = e0_;
    } else {
      e0_ = 0;
      e1_ = 0;
      s_ = -1;
    }
  }

  /** \brief Computes and sets the Shi-Tomasi Score \ref s_, considering only one specific pyramid level.
   *
   * @param l - Considered pyramid level.
   */
  void getSpecificLevelScore(const int l){
    if(isValidPatch_[l]){
      s_ = patches_[l].getScore();
    } else {
      s_ = -1;
    }
  }

  /** \brief Draws the patches of the MultilevelPatchFeature into an image.
   *
   * @param drawImg    - Image in which should be drawn.
   * @param c          - Center pixel coordinates of the patch (on level 0)
   * @param stretch    - %Patch drawing magnification factor.
   * @param withBorder - Draw either the patches Patch::patch_ (withBorder = false) or the expanded patches
   *                     Patch::patchWithBorder_ (withBorder = true) .
   */
  void drawMultilevelPatch(cv::Mat& drawImg,const cv::Point2i& c,int stretch = 1,const bool withBorder = false) const{
    for(int l=nLevels_-1;l>=0;l--){
      if(isValidPatch_[l]){
        cv::Point2i corner = cv::Point2i((patch_size/2+(int)withBorder)*(pow(2,nLevels_-1)-pow(2,l)),(patch_size/2+(int)withBorder)*(pow(2,nLevels_-1)-pow(2,l)));
        patches_[l].drawPatch(drawImg,c+corner,stretch*pow(2,l),withBorder);
      }
    }
  }

  /** \brief Computes the RMSE (Root Mean Squared Error) with respect to the patches of an other MultilevelPatchFeature
   *         for an specific pyramid level interval.
   *
   * @param mlp        - \ref MultilevelPatchFeature, which patches should be used for the RMSE computation.
   * @param l1         - Start pyramid level (l1<l2)
   * @param l2         - End pyramid level (l1<l2)
   * @return the RMSE value for the patches in the set pyramid level interval.
   */
  float computeAverageDifference(const MultilevelPatchFeature<n_levels,patch_size>& mlp, const int l1, const int l2) const{
    float offset = 0.0f;
    for(int l = l1; l <= l2; l++){
      const float* it_patch = patches_[l].patch_;
      const float* it_patch_in = mlp.patches_[l].patch_;
      for(int y=0; y<patch_size; ++y){
        for(int x=0; x<patch_size; ++x, ++it_patch, ++it_patch_in){
          offset += *it_patch - *it_patch_in;
        }
      }
    }
    offset = offset/(patch_size*patch_size*(l2-l1+1));
    float error = 0.0f;
    for(int l = l1; l <= l2; l++){
      const float* it_patch = patches_[l].patch_;
      const float* it_patch_in = mlp.patches_[l].patch_;
      for(int y=0; y<patch_size; ++y){
        for(int x=0; x<patch_size; ++x, ++it_patch, ++it_patch_in){
          error += std::pow(*it_patch - *it_patch_in - offset,2);
        }
      }
    }
    error = error/(patch_size*patch_size*(l2-l1+1));
    return std::sqrt(error);
  }

  /** \brief Computes the RMSE (Root Mean Squared Error) with respect to the patch extracted from the reference image
   *         for an specific pyramid level interval.
   *
   * @param pyr        - Image pyramid of reference image
   * @param l1         - Start pyramid level (l1<l2)
   * @param l2         - End pyramid level (l1<l2)
   * @return the RMSE value for the patches in the set pyramid level interval.
   */
  float computeAverageDifferenceReprojection(const ImagePyramid<n_levels>& pyr, const int l1, const int l2, const bool doWarping = false) const{
    MultilevelPatchFeature<n_levels,patch_size> mlpReprojected;
    mlpReprojected.set_c(get_c());
    if(doWarping)
      mlpReprojected.set_affineTransfrom(get_affineTransform());
    extractMultilevelPatchFromImage(mlpReprojected,pyr,l2,false,doWarping);
    return computeAverageDifference(mlpReprojected,l1,l2);
  }
};

/** \brief Checks if the MultilevelPatchFeature's patches are fully located within the corresponding images.
 *
 * @tparam n_levels   - Number of pyramid levels.
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Image pyramid, which should be checked to fully contain the patches.
 * @param l           - Maximal pyramid level which should be checked (Note: The maximal level is the critical level.)
 * @param withBorder  - If true, the check is executed with the expanded patch dimensions (incorporates the general patch dimensions).
 *                      If false, the check is only executed with the general patch dimensions.
 * @param doWarping   - If true, the specified affine transformation in mlp is used to extract the (warped) patch dimensions for the check.
 *                      If false, the dimensions of aligned patches are used for the check.
 */
template<int n_levels,int patch_size>
bool isMultilevelPatchInFrame(const MultilevelPatchFeature<n_levels,patch_size>& mlp,const ImagePyramid<n_levels>& pyr, const int l = n_levels-1,const bool withBorder = false, const bool doWarping = false){
  if(!mlp.isInFront()) return false;
  const cv::Point2f c = levelTranformCoordinates(mlp.get_c(),pyr,0,l);
  if(!doWarping){
    return isPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c,withBorder);
  } else {
    return isWarpedPatchInFrame(mlp.patches_[l],pyr.imgs_[l],c,mlp.get_affineTransform(),withBorder);
  }
}

/** \brief Extracts a multilevel patch from a given image pyramid.
 *
 * @tparam n_levels   - Number of pyramid levels.
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which finally holds the multilevel patch information.
 *                      Has to be initialized, such that it holds already the patch dimension information!
 * @param pyr         - Image pyramid from which the patch data should be extracted.
 * @param l           - Patches are extracted from pyramid level 0 to l.
 * @param withBorder  - If true, both, the general patches and the corresponding expanded patches are extracted.
 * @param doWarping   - If true, the specified affine transformation in mlp is used to extract the (warped) patches.
 *                      If false, patches are extracted, which are aligned with the image axes.
 */
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Transforms pixel coordinates between two pyramid levels.
 *
 * @tparam n_levels  - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @param c          - Pixel coordinates on pyramid level l1.
 * @param pyr        - Considered image pyramid.
 * @param l1         - Original pyramid level.
 * @param l2         - Target pyramid level.
 * @return the corresponding pixel coordinates on pyramid level l2.
 */
template<int n_levels>
cv::Point2f levelTranformCoordinates(const cv::Point2f c,const ImagePyramid<n_levels>& pyr,const int l1, const int l2){
  assert(l1<n_levels && l2<n_levels);
  return (pyr.centers_[l1]-pyr.centers_[l2])*pow(0.5,l2)+c*pow(0.5,l2-l1);
}


/** \brief Draws the patch borders into an image.
 *
 * @tparam n_levels   - Total number of pyramid levels.
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param drawImg     - Image in which the patch borders should be drawn.
 * @param s           - Scaling factor.
 * @param color       - Line color.
 */
template<int n_levels,int patch_size>
static void drawPatchBorder(cv::Mat& drawImg, MultilevelPatchFeature<n_levels,patch_size>& mlp, const float s, const cv::Scalar& color){
  const PixelCorners& pixelCorners = mlp.get_pixelCorners();
  const cv::Point2f& center = mlp.get_c();
  FeatureCoordinates fc1;
  FeatureCoordinates fc2;
  fc1.set_c(center - s*pixelCorners[0] - s*pixelCorners[1]);
  fc2.set_c(center + s*pixelCorners[0] - s*pixelCorners[1]);
  drawLine(drawImg,fc1,fc2,color,1);
  fc1.set_c(center + s*pixelCorners[0] + s*pixelCorners[1]);
  drawLine(drawImg,fc1,fc2,color,1);
  fc2.set_c(center - s*pixelCorners[0] + s*pixelCorners[1]);
  drawLine(drawImg,fc1,fc2,color,1);
  fc1.set_c(center - s*pixelCorners[0] - s*pixelCorners[1]);
  drawLine(drawImg,fc1,fc2,color,1);
}

/** \brief Get the raw linear align equations (A*x=b), given by the [(#pixel)x2] Matrix  A (float) and the [(#pixel)x1] vector b (float).
 *
 *  \see MultilevelPatchFeature::A_ and MultilevelPatchFeature::b_.
 *  \see Function getLinearAlignEquationsReduced() to get an optimized linear align equations.
 *
 * @tparam n_levels   - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Considered image pyramid.
 * @param l1          - Start pyramid level (l1<l2)
 * @param l2          - End pyramid level (l1<l2)
 * @param doWarping   - Should warping be considered
 * @param A           - Jacobian of the pixel intensities w.r.t. to pixel coordinates
 * @param b           - Intensity errors
 * @return true, if successful.
 */
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


/** \brief Get the reduced (QR-decomposition) linear align equations (A*x=b), given by the [2x2] Matrix A (float)
 *         and the [2x1] vector b (float).
 *
 *  \see MultilevelPatchFeature::A_ and MultilevelPatchFeature::b_.
 *  \see Function getLinearAlignEquations() to get the raw linear align equations.
 *  \see Function getLinearAlignEquationsReduced(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr, const int l1, const int l2, const bool doWarping, Eigen::Matrix2d& A_red, Eigen::Vector2d& b_red)
 *                to get the reduced linear align equations in __double__ precision.
 *
 * @tparam n_levels   - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Considered image pyramid.
 * @param l1          - Start pyramid level (l1<l2)
 * @param l2          - End pyramid level (l1<l2)
 * @param doWarping   - Should warping be considered
 * @param A_red       - Reduced Jacobian of the pixel intensities w.r.t. to pixel coordinates
 * @param b_red       - Reduced intensity errors
 * @return true, if successful.
 */
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

/** \brief Get the reduced (QR-decomposition) linear align equations (A*x=b), given by the [2x2] Matrix A (double)
 *         and the [2x1] vector b (double).
 *
 *         \see MultilevelPatchFeature::A_ and MultilevelPatchFeature::b_.
 *         \see Function getLinearAlignEquations() to get the raw linear align equations.
 *         \see Function getLinearAlignEquationsReduced(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr, const int l1, const int l2, const bool doWarping, Eigen::Matrix2f& A_red, Eigen::Vector2f& b_red)
 *                       to get the reduced linear align equations in __float__ precision.
 *
 * @tparam n_levels   - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Considered image pyramid.
 * @param l1          - Start pyramid level (l1<l2)
 * @param l2          - End pyramid level (l1<l2)
 * @param doWarping   - Should warping be considered
 * @param A_red       - Reduced Jacobian of the pixel intensities w.r.t. to pixel coordinates
 * @param b_red       - Reduced intensity errors
 * @return true, if successful.
 */
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

/** \brief Inverse compositional 2D patch alignment (old)
 *
 * @tparam n_levels   - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Considered image pyramid.
 * @param l1          - Start pyramid level (l1<l2)
 * @param l2          - End pyramid level (l1<l2)
 * @param doWarping   - Should warping be considered
 * @param maxIter     - Maximal number of iterations
 * @param minPixUpd   - Termination condition on absolute pixel update
 * @return true, if alignment converged!
 */
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

/** \brief 2D patch alignment.
 *
 * @tparam n_levels   - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Considered image pyramid.
 * @param l1          - Start pyramid level (l1<l2)
 * @param l2          - End pyramid level (l1<l2)
 * @param doWarping   - Should warping be considered
 * @param maxIter     - Maximal number of iterations
 * @param minPixUpd   - Termination condition on absolute pixel update
 * @return true, if alignment converged!
 */
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

/** \brief Execute a 2D patch alignment using only one single pyramid level (patch) of the MultilevelPatchFeature.
 *
 * @tparam n_levels   - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Considered image pyramid.
 * @param l           - Pyramid level which is used for the alignement
 * @param doWarping   - Should warping be considered
 * @return true, if alignment converged!
 *
 * @todo check and complete this
 */
template<int n_levels,int patch_size>
bool align2DSingleLevel(MultilevelPatchFeature<n_levels,patch_size>& mlp, const ImagePyramid<n_levels>& pyr, const int l, const bool doWarping){
  return align2D(mlp,pyr,l,l,doWarping);
}

/** \brief Aligns a MultilevelPatchFeature to a given image pyramid and updates its matching status
 *         (MultilevelPatchFeature::status_) and its position (MultilevelPatchFeature::c_).
 *
 * @tparam n_levels   - Total number of pyramid levels. Note : The value of l1 and l2 has to be smaller then n_levels!
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @param mlp         - \ref MultilevelPatchFeature, which contains the patches.
 * @param pyr         - Considered image pyramid.
 * @param start_level - Starting pyramid level for alignment
 * @param end_level   - Ending pyramif level for alignment (should be smaller than start_level)
 * @param num_seq     - On how many levels should the alignment be carried out synchroneously @todo: correct algorithm
 * @param doWarping   - Should warping be considered
 */
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, storing and handling a set of MultilevelPatchFeature%s.
 *
 * @tparam n_levels   - Total number of pyramid levels for each MultilevelPatchFeature in the set.
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @tparam nMax       - Maximum number of MultilevelPatchFeature in the set.
 */
template<int n_levels,int patch_size,int nMax>
class MultilevelPatchSet{
 public:
  MultilevelPatchFeature<n_levels,patch_size> features_[nMax];  /**<Array of  MultilevelPatchFeature.*/
  bool isValid_[nMax];  /**<Array, defining if there is a valid MultilevelPatchFeature at the considered array index. */
  int maxIdx_;  /**<Current maximum array/set index. Number of MultilevelPatchFeature, which have already been inserted into the set. */

  /** \brief Constructor
   */
  MultilevelPatchSet(){
    reset();
  }

  /** \brief Destructor
     */
  ~MultilevelPatchSet(){}

  /** \brief Resets the MultilevelPatchSet.
   */
  void reset(){
    maxIdx_ = 0;
    for(unsigned int i=0;i<nMax;i++){
      features_[i].setCamera(nullptr);
      isValid_[i] = false;
    }
  }

  /** \brief Resets the MultilevelPatchSet.
   *
   * @param ind - Free array/set index.
   * @return true, if a free index was found.
   */
  bool getFreeIndex(int& ind) const{
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == false){
        ind = i;
        return true;
      }
    }
    return false;
  }

  /** \brief Get the number of valid MultilevelPatchFeature in the set.
   *
   * @return the number of valid MultilevelPatchFeature in the set.
   */
  int getValidCount() const{
    int count = 0;
    for(unsigned int i=0;i<nMax;i++){
      if(isValid_[i] == true) ++count;
    }
    return count;
  }

  /** \brief Add a MultilevelPatchFeature to the set.
   *
   * @param feature - MultilevelPatchFeature which should be inserted into the set.
   * @param camID   - Corresponding camera ID.
   * @return the array/set index, where the MultilevelPatchFeature has been stored. The value -1 is returned,
   *         if the feature could not be inserted, because the maximal number of features have already been reached.
   */
  int addFeature(const MultilevelPatchFeature<n_levels,patch_size>& feature, const int camID){
    int newInd = -1;
    if(getFreeIndex(newInd)){
      features_[newInd] = feature;
      features_[newInd].idx_ = maxIdx_++;
      features_[newInd].camID_ = camID;
      isValid_[newInd] = true;
    } else {
      std::cout << "Feature Manager: maximal number of feature reached" << std::endl;
    }
    return newInd;
  }

  /** \brief Get the average Shi-Tomasi Score of all MultilevelPatchFeature in the set.
   *
   * @return the average Shi-Tomasi Score of all MultilevelPatchFeature in the set.
   */
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

/** \brief Adds the best MultilevelPatchFeature%s from a candidates list to an existing MultilevelPatchSet.
 *
 *  This function takes a given feature candidate list and builds, in a first step,
 *  MultilevelPatchFeature%s out of it using a given image pyramid. For each MultilevelPatchFeature the
 *  corresponding Shi-Tomasi Score is computed.
 *  In a second step, the candidate MultilevelPatchFeature%s are sorted and
 *  placed into buckets, depending on their individual Shi-Tomasi Score. MultilevelPatchFeature%s in a high bucket
 *  (high bucket index) have higher Shi-Tomasi Scores than MultilevelPatchFeature%s which have been placed into a
 *  low bucket.
 *  In a third step, the MultilevelPatchFeature%s in the buckets are reordered, depending on their distance
 *  to already existing features in the given MultilevelPatchSet. A small distance to an existing feature is punished,
 *  by moving the concerned candidate MultilevelPatchFeature into a lower bucket.
 *  Finally the existing MultilevelPatchSet is expanded with the best (high bucket index) candidate MultilevelPatchFeature%s.
 *
 * @tparam n_levels   - Total number of pyramid levels for each MultilevelPatchFeature.
 * @tparam patch_size - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @tparam nMax       - Maximum number of MultilevelPatchFeature%s in the MultilevelPatchSet.
 *
 * @param mlpSet                 - MultilevelPatchSet which should be expanded/filled with the best MultilevelPatchFeature%s from the candidates list.
 * @param candidates             - List of candidate feature coordinates.
 * @param pyr                    - Image pyramid used to extract the MultilevelPatchFeature%s from the candidates list.
 * @param camID                  - %Camera ID
 * @param initTime               - Current time (time at which the MultilevelPatchFeature%s are created from the candidates list).
 * @param l1                     - Start pyramid level for the Shi-Tomasi Score computation of MultilevelPatchFeature%s extracted from the candidates list.
 * @param l2                     - End pyramid level for the Shi-Tomasi Score computation of MultilevelPatchFeature%s extracted from the candidates list.
 * @param maxN                   - Maximal number of features which should be added to the mlpSet.
 * @param nDetectionBuckets      - Number of buckets.
 * @param scoreDetectionExponent - Choose it between [0 1]. 1 : Candidate features are sorted linearly into the buckets, depending on their Shi-Tomasi score.
 *                                                          0 : All candidate features are filled into the highest bucket.
 *                                 A small scoreDetectionExponent forces more candidate features into high buckets.
 * @param penaltyDistance        - If a candidate feature has a smaller distance to an existing feature in the mlpSet, it is punished (shifted in an lower bucket) dependent of its actual distance to the existing feature.
 * @param zeroDistancePenalty    - A candidate feature in a specific bucket is shifted zeroDistancePenalty-buckets back to a lower bucket if it has zero distance to an existing feature in the mlpSet.
 * @param requireMax             - Should the adding of maxN be enforced?
 * @param minScore               - Shi-Tomasi Score threshold for the best (highest Shi-Tomasi Score) MultilevelPatchFeature extracted from the candidates list.
 *                                 If the best MultilevelPatchFeature has a Shi-Tomasi Score less than or equal this threshold, the function aborts and returns an empty map.
 *
 * @return an unordered_set, holding the indizes of the MultilevelPatchSet, at which the new MultilevelPatchFeature%s have been added (from the candidates list).
 */
// TODO: work more on bearing vectors (in general)
template<int n_levels,int patch_size,int nMax>
std::unordered_set<unsigned int> addBestCandidates(MultilevelPatchSet<n_levels,patch_size,nMax>& mlpSet, std::list<cv::Point2f>& candidates, const ImagePyramid<n_levels>& pyr, const int camID, const double initTime,
                                                   const int l1, const int l2, const int maxN, const int nDetectionBuckets, const double scoreDetectionExponent,
                                                   const double penaltyDistance, const double zeroDistancePenalty, const bool requireMax, const float minScore){
  std::unordered_set<unsigned int> newSet;
  std::list<MultilevelPatchFeature<n_levels,patch_size>> candidatesWithPatch;

  // Create MultilevelPatchFeature from the candidates list and compute their Shi-Tomasi Score.
  float maxScore = -1.0;
  for(auto it = candidates.begin(); it != candidates.end(); ++it){
    candidatesWithPatch.emplace_back();
    candidatesWithPatch.rbegin()->reset(-1,initTime);
    candidatesWithPatch.rbegin()->set_c(*it);
    if(isMultilevelPatchInFrame(*candidatesWithPatch.rbegin(),pyr,n_levels-1,true)){
      extractMultilevelPatchFromImage(*candidatesWithPatch.rbegin(),pyr,n_levels-1,true);
      candidatesWithPatch.rbegin()->computeMultilevelShiTomasiScore(l1,l2);
      if(candidatesWithPatch.rbegin()->s_ > maxScore) maxScore = candidatesWithPatch.rbegin()->s_;
    }
    else {
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
  FeatureCoordinates featureCoordinates;
  for(unsigned int i=0;i<nMax;i++){
    if(mlpSet.isValid_[i]){
      mpFeature = &mlpSet.features_[i];
      featureCoordinates = static_cast<FeatureCoordinates>(*mpFeature);
      featureCoordinates.set_nor(featureCoordinates.get_nor_other(camID));
      featureCoordinates.camID_ = camID;
      if(featureCoordinates.isInFront()){
        for (unsigned int bucketID = 1;bucketID < nDetectionBuckets;bucketID++) {
          for (auto it_cand = buckets[bucketID].begin();it_cand != buckets[bucketID].end();) {
            doDelete = false;
            d2 = std::pow(featureCoordinates.get_c().x - (*it_cand)->c_.x,2) + std::pow(featureCoordinates.get_c().y - (*it_cand)->c_.y,2);  // Squared distance between the existing feature and the candidate feature.
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
  }

  // Incrementally add features and update candidate buckets (Check distance of candidates with respect to the
  // newly inserted feature).
  MultilevelPatchFeature<n_levels,patch_size>* mpNewFeature;
  int addedCount = 0;
  for (int bucketID = nDetectionBuckets-1;bucketID >= 0+static_cast<int>(!requireMax);bucketID--) {
    while(!buckets[bucketID].empty() && addedCount < maxN && mlpSet.getValidCount() != nMax) {
      mpNewFeature = *(buckets[bucketID].begin());
      buckets[bucketID].erase(mpNewFeature);
      const int ind = mlpSet.addFeature(*mpNewFeature,camID);
      if(ind >= 0){
        newSet.insert(ind);
      }
      addedCount++;
      for (unsigned int bucketID2 = 1;bucketID2 <= bucketID;bucketID2++) {
        for (auto it_cand = buckets[bucketID2].begin();it_cand != buckets[bucketID2].end();) {
          doDelete = false;
          d2 = std::pow(mpNewFeature->get_c().x - (*it_cand)->c_.x,2) + std::pow(mpNewFeature->get_c().y - (*it_cand)->c_.y,2);
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Extract FastCorner coordinates from an Image Pyramid.
 *
 * @tparam n_levels          - Total number of levels of the image pyramid.
 *
 * @param pyr                - Image pyramid, which serves for the corner extraction.
 * @param candidates         - List of the extracted corner coordinates (defined on pyramid level 0).
 * @param l                  - Pyramid level at which the corners should be extracted.
 * @param detectionThreshold - Detection threshold of the used cv::FastFeatureDetector.
 *                             See http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
template <int n_levels>
void detectFastCorners(const ImagePyramid<n_levels>& pyr, std::list<cv::Point2f>& candidates, int l, int detectionThreshold) {
  std::vector<cv::KeyPoint> keypoints;
  cv::FastFeatureDetector feature_detector_fast(detectionThreshold, true);
  feature_detector_fast.detect(pyr.imgs_[l], keypoints);
  cv::Point2f level_c;
  for (auto it = keypoints.cbegin(), end = keypoints.cend(); it != end; ++it) {
    level_c = cv::Point2f(it->pt.x, it->pt.y);
    candidates.push_back(levelTranformCoordinates(level_c,pyr,l,0));
  }
}

/** \brief Prunes the (corner/feature) candidate list by erasing candidates, which are too close at an existing feature.
 *
 * @tparam n_levels          - Total number of levels of the image pyramid.
 * @tparam patch_size        - Edge length of the patches in pixels. Value must be a multiple of 2!
 * @tparam nMax              - Maximum number of MultilevelPatchFeature%s in the MultilevelPatchSet.
 *
 * @param mlpSet             - MultilevelPatchSet, containing the existing MultilevelPatchFeature%s .
 * @param candidates         - List of extracted corner coordinates/candidates (defined on pyramid level 0).
 * @param candidateID        - Camera ID, in which the candidates have been extracted.
 */
template<int n_levels,int patch_size,int nMax>
void pruneCandidates(const MultilevelPatchSet<n_levels,patch_size,nMax>& mlpSet, std::list<cv::Point2f>& candidates, const int candidateID){ // TODO: add corner motion dependency
  constexpr float t2 = patch_size*patch_size;  // TODO: param
  bool prune;
  const MultilevelPatchFeature<n_levels,patch_size>* mpFeature;
  FeatureCoordinates featureCoordinates;
  for (auto it = candidates.begin(); it != candidates.end();) {
    prune = false;
    for(unsigned int i=0;i<nMax;i++){
      if(mlpSet.isValid_[i]){
        mpFeature = &mlpSet.features_[i];
        featureCoordinates = static_cast<FeatureCoordinates>(*mpFeature);
        featureCoordinates.set_nor(featureCoordinates.get_nor_other(candidateID));
        featureCoordinates.camID_ = candidateID;
        if(featureCoordinates.isInFront() && pow(it->x-featureCoordinates.get_c().x,2) + pow(it->y-featureCoordinates.get_c().y,2) < t2){ // TODO: check inFrame, only if covariance not too large
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Get the depth value from the triangulation of two bearing vectors.
 *
 *  @param C1fP    - Bearing vector in the reference frame C1 (unit length!).
 *  @param C2fP    - Bearing vector in another frame C2 (unit length!).
 *  @param C2rC2C1 - Position vector, pointing from C2 to C1, expressed in cooridantes of C2.
 *  @param qC2C1   - Quaternion, expressing the orientation of C1 in the C2.
 *  @param d       - Triangulated depth value along the bearing vector C1fP.
 *  @return true, if triangulation successful. This means the angle between the projection rays has not been too small.
 */
bool getDepthFromTriangulation(const V3D& C1fP, const V3D& C2fP, const V3D& C2rC2C1, const QPD& qC2C1, double* d)
{
  Eigen::Matrix<double,3,2> B;
  B <<  qC2C1.rotate(C1fP), C2fP;
  const Eigen::Matrix2d BtB = B.transpose() * B;
  if(BtB.determinant() < 0.000001)
    return false;                      // Projection rays almost parallel.
  const Eigen::Vector2d dv = - BtB.inverse() * B.transpose() * C2rC2C1;
  *d = fabs(dv[0]);
  return true;
}

/** \brief Get the depth uncertainty tau of a triangulated depth value.
 *
 *  Consider a bearing vector C1fP in a reference frame and a bearing vector C2fP in a partner frame have been used
 *  to triangulate a depth value d (along the bearing vector C1fP). Let's call the so gained 3D landmark position P.
 *  In order to get depth uncertainty value of d (along the bearing vector C1fP), a constant pixel error
 *  of the detection of C2fP can be projected to the ray of C1fP (to the side which is farther to the reference frame).
 *  Let's call 3D point corresponding to the maximal pixel error P_plus.
 *  The depth uncertainty tau is then defined as \f$tau=|(P\_plus - P)|\f$.
 *
 *  @param C1fP           - Bearing vector in the reference frame (unit length!).
 *  @param C2fP           - Bearing vector in another frame (unit length!).
 *  @param C2rC2C1        - Position vector, pointing from C2 to C1, expressed in cooridantes of C2.
 *  @param d              - Triangulated depth value along the bearing vector C1fP.
 *  @param px_error_angle - Angle between the bearing vector C2fP and the bearing vector corresponding to the maximal
 *                          pixel error. <br>
 *                          Compute it as: <br>
 *                           \f$px\_error\_angle = 2 \cdot \arctan{\frac{px\_error}{2 \cdot fx}}\f$ <br>
 *                          ,where px_error is the assumed pixel error (e.g. 1 pixel) and
 *                          fx the focal length (expressed in pixel) of the camera belonging to C2fP.
 * @return the depth uncertainty value \f$tau=|(P\_plus - P)|\f$.
 */
float getDepthUncertaintyTau(const V3D& C1fP, const V3D& C1rC1C2, const float d, const float px_error_angle)
{
  float t_0 = C1rC1C2(0);
  float t_1 = C1rC1C2(1);
  float t_2 = C1rC1C2(2);
  float a_0 = C1fP(0) * d - t_0;
  float a_1 = C1fP(1) * d - t_1;
  float a_2 = C1fP(2) * d - t_2;
  float t_norm = std::sqrt(t_0 * t_0 + t_1 * t_1 + t_2 * t_2);
  float a_norm = std::sqrt(a_0 * a_0 + a_1 * a_1 + a_2 * a_2);
  float alpha = std::acos((C1fP(0) * t_0 + C1fP(1) * t_1 + C1fP(2) * t_2) / t_norm);
  float beta = std::acos(( a_0 * (-t_0) + a_1 * (-t_1) + a_2 * (-t_2) ) / (t_norm * a_norm));
  float beta_plus = beta + px_error_angle;
  float gamma_plus = M_PI - alpha - beta_plus;                             // Triangle angles sum to PI.
  float d_plus = t_norm * std::sin(beta_plus) / std::sin(gamma_plus);      // Law of sines.
  return (d_plus - d);                                                     // Tau.
}



}


#endif /* ROVIO_COMMON_VISION_HPP_ */
