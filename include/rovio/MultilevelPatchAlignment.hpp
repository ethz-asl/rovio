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

#ifndef ROVIO_MULTILEVELPATCHALIGNMENT_HPP_
#define ROVIO_MULTILEVELPATCHALIGNMENT_HPP_

#include "lightweight_filtering/common.hpp"
#include "rovio/ImagePyramid.hpp"
#include "rovio/MultilevelPatch.hpp"
#include "rovio/FeatureCoordinates.hpp"
#include "rovio/FeatureWarping.hpp"

namespace rovio{

/** \brief %Class for handling the alignement of multilevel patches
 *
 *   @tparam nLevels - Total number of pyramid levels.
 *   @tparam patch_size - Edge length of the patch in pixels. Value must be a multiple of 2!
 */
template<int nLevels,int patch_size>
class MultilevelPatchAlignment {
 public:
  mutable Eigen::MatrixXf A_;  /**<A matrix of the linear system of equations, needed for the multilevel patch alignment.*/
  mutable Eigen::MatrixXf b_;  /**<b matrix/vector of the linear system of equations, needed for the multilevel patch alignment.*/
  mutable Eigen::ColPivHouseholderQR<Eigen::MatrixXf> mColPivHouseholderQR_;  /**<QR decomposition module. Used for computiong reduces system of equations.*/
  mutable Eigen::JacobiSVD<Eigen::MatrixXf> svd_; /**<SVD module. Used for solving linear equation systems.*/
  mutable FeatureCoordinates bestCoordinateMatch_; /**<Best current pixel coordinate match.*/
  mutable double bestIntensityError_; /**<Intensity error for the match.*/
  mutable MultilevelPatch<nLevels,patch_size> mlpTemp_; /**<Temporary multilevel patch used for various computations.*/

  /** \brief Destructor
   */
  virtual ~MultilevelPatchAlignment(){};

  /** \brief Get the raw linear align equations (A*x=b), given by the [(#pixel)x2] Matrix  A (float) and the [(#pixel)x1] vector b (float).
   *
   *  \see MultilevelPatchFeature::A_ and MultilevelPatchFeature::b_.
   *  \see Function getLinearAlignEquationsReduced() to get an optimized linear align equations.
   *
   * @param pyr         - Considered image pyramid.
   * @param mp          - \ref MultilevelPatch, which contains the patches.
   * @param c           - Coordinates of the patch in the reference image.
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param l1          - Start pyramid level (l1<l2)
   * @param l2          - End pyramid level (l1<l2)
   * @param A           - Jacobian of the pixel intensities w.r.t. to pixel coordinates
   * @param b           - Intensity errors
   * @return true, if successful.
   * @todo catch if warping too distorted
   */
  bool getLinearAlignEquations(const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& c, const FeatureWarping* mpWarp, const int l1, const int l2,
                               Eigen::MatrixXf& A, Eigen::MatrixXf& b){
    A.resize(0,0);
    b.resize(0,0);
    Eigen::Matrix2f W;
    Eigen::Matrix2f affInv;
    if(!c.com_c() || (mpWarp != nullptr && !mpWarp->com_affineTransform(&c))){
      return false;
    }
    if(mpWarp != nullptr){
      W = mpWarp->get_affineTransform(&c);
    } else {
      W.setIdentity();
    }
    affInv = W.inverse();
    int numLevel = 0;
    FeatureCoordinates c_level;
    const int halfpatch_size = patch_size/2;
    float mean_diff = 0;
    float mean_diff_dx = 0;
    float mean_diff_dy = 0;
    Patch<patch_size> extractedPatch;
    for(int l = l1; l <= l2; l++){
      pyr.levelTranformCoordinates(c,c_level,0,l);
      if(mp.isValidPatch_[l] && extractedPatch.isPatchInFrame(pyr.imgs_[l],c_level.get_c(),W,false)){
        mp.patches_[l].computeGradientParameters();
        if(mp.patches_[l].validGradientParameters_){
          numLevel++;
          A.conservativeResize(numLevel*patch_size*patch_size,2);
          b.conservativeResize(numLevel*patch_size*patch_size,1);
          const int refStep = pyr.imgs_[l].step.p[0];
          const float* it_patch = mp.patches_[l].patch_;
          const float* it_dx = mp.patches_[l].dx_;
          const float* it_dy = mp.patches_[l].dy_;
          extractedPatch.extractPatchFromImage(pyr.imgs_[l],c_level.get_c(),W,false);
          const float* it_patch_extracted = extractedPatch.patch_;
          for(int y=0; y<patch_size; ++y){
            for(int x=0; x<patch_size; ++x, ++it_patch, ++it_patch_extracted, ++it_dx, ++it_dy){
              const float res = *it_patch_extracted - *it_patch;
              const float Jx = -pow(0.5,l)*(*it_dx);
              const float Jy = -pow(0.5,l)*(*it_dy);
              mean_diff += res;
              b((numLevel-1)*patch_size*patch_size+y*patch_size+x,0) = res;
              if((W-Eigen::Matrix2f::Identity()).norm() < 1e-6){
                mean_diff_dx += Jx;
                mean_diff_dy += Jy;
                A((numLevel-1)*patch_size*patch_size+y*patch_size+x,0) = Jx;
                A((numLevel-1)*patch_size*patch_size+y*patch_size+x,1) = Jy;
              } else {
                const float Jx_warp = Jx*affInv(0,0)+Jy*affInv(1,0);
                const float Jy_warp = Jx*affInv(0,1)+Jy*affInv(1,1);
                mean_diff_dx += Jx_warp;
                mean_diff_dy += Jy_warp;
                A((numLevel-1)*patch_size*patch_size+y*patch_size+x,0) = Jx_warp;
                A((numLevel-1)*patch_size*patch_size+y*patch_size+x,1) = Jy_warp;
              }
            }
          }
        }
      }
    }
    if(numLevel==0){
      return false;
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
   *  \see Function getLinearAlignEquationsReduced(MultilevelPatchFeature<nLevels,patch_size>& mlp, const ImagePyramid<nLevels>& pyr, const int l1, const int l2, const bool doWarping, Eigen::Matrix2d& A_red, Eigen::Vector2d& b_red)
   *                to get the reduced linear align equations in __double__ precision.
   *
   * @param pyr         - Considered image pyramid.
   * @param mp          - \ref MultilevelPatch, which contains the patches.
   * @param c           - Coordinates of the patch in the reference image.
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param l1          - Start pyramid level (l1<l2)
   * @param l2          - End pyramid level (l1<l2)
   * @param A_red       - Reduced Jacobian of the pixel intensities w.r.t. to pixel coordinates
   * @param b_red       - Reduced intensity errors
   * @return true, if successful.
   */
  bool getLinearAlignEquationsReduced(const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& c, const FeatureWarping* mpWarp, const int l1, const int l2,
                                      Eigen::Matrix2f& A_red, Eigen::Vector2f& b_red){
    bool success = getLinearAlignEquations(pyr,mp,c,mpWarp,l1,l2,A_,b_);
    if(success){
      mColPivHouseholderQR_.compute(A_);
      b_red = (mColPivHouseholderQR_.householderQ().inverse()*b_).template block<2,1>(0,0); // TODO: reduce
      A_red = mColPivHouseholderQR_.matrixR().template block<2,2>(0,0);
      A_red(1,0) = 0.0;
      A_red = A_red*mColPivHouseholderQR_.colsPermutation();
    }
    return success;
  }

  /** \brief Get the reduced (QR-decomposition) linear align equations (A*x=b), given by the [2x2] Matrix A (double)
   *         and the [2x1] vector b (double).
   *
   *         \see MultilevelPatchFeature::A_ and MultilevelPatchFeature::b_.
   *         \see Function getLinearAlignEquations() to get the raw linear align equations.
   *         \see Function getLinearAlignEquationsReduced(MultilevelPatchFeature<nLevels,patch_size>& mlp, const ImagePyramid<nLevels>& pyr, const int l1, const int l2, const bool doWarping, Eigen::Matrix2f& A_red, Eigen::Vector2f& b_red)
   *                       to get the reduced linear align equations in __float__ precision.
   *
   * @param pyr         - Considered image pyramid.
   * @param mp          - \ref MultilevelPatch, which contains the patches.
   * @param c           - Coordinates of the patch in the reference image.
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param l1          - Start pyramid level (l1<l2)
   * @param l2          - End pyramid level (l1<l2)
   * @param A_red       - Reduced Jacobian of the pixel intensities w.r.t. to pixel coordinates
   * @param b_red       - Reduced intensity errors
   * @return true, if successful.
   */
  bool getLinearAlignEquationsReduced(const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& c, const FeatureWarping* mpWarp, const int l1, const int l2,
                                      Eigen::Matrix2d& A_red, Eigen::Vector2d& b_red){
    Eigen::Matrix2f A_red_;
    Eigen::Vector2f b_red_;
    bool success = getLinearAlignEquationsReduced(pyr,mp,c,mpWarp,l1,l2,A_red_,b_red_);
    if(success){
      A_red = A_red_.cast<double>();
      b_red = b_red_.cast<double>();
    }
    return success;
  }

  /** \brief Inverse compositional 2D patch alignment (old version, used for unit testing)
   *
   * @param cOut        - Estimated coordinates for the patch alignment.
   * @param pyr         - Considered image pyramid.
   * @param mp          - \ref MultilevelPatch, which contains the patches.
   * @param cInit       - Coordinates of the patch in the reference image, initial guess.
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param l1          - Start pyramid level (l1<l2)
   * @param l2          - End pyramid level (l1<l2)
   * @param maxIter     - Maximal number of iterations
   * @param minPixUpd   - Termination condition on absolute pixel update
   * @return true, if alignment converged!
   * @todo catch if warping too distorted
   */
  bool align2D_old(FeatureCoordinates& cOut, const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& cInit, const FeatureWarping* mpWarp, const int l1, const int l2,
                   const int maxIter = 10, const double minPixUpd = 0.03){
    Eigen::Matrix2f W;
    Eigen::Matrix2f WInv;
    if(!cInit.com_c() || (mpWarp != nullptr && !mpWarp->com_affineTransform(&cInit))){
      return false;
    }
    if(mpWarp != nullptr){
      W = mpWarp->get_affineTransform(&cInit);
    } else {
      W.setIdentity();
    }
    WInv = W.inverse();
    int numLevel = 0;
    FeatureCoordinates c_level;
    for(int l = l1; l <= l2; l++){
      pyr.levelTranformCoordinates(cInit,c_level,0,l);
      if(mp.isValidPatch_[l] && mp.patches_[l].isPatchInFrame(pyr.imgs_[l],c_level.get_c(),W,false)){
        mp.patches_[l].computeGradientParameters();
        if(mp.patches_[l].validGradientParameters_){
          numLevel++;
        }
      }
    }
    if(numLevel==0){
      return false;
    }
    Eigen::Matrix3f aff = Eigen::Matrix3f::Identity();
    if(mpWarp != nullptr){
      aff.block<2,2>(0,0) = W;
    }
    Eigen::Matrix3f affInv = aff.inverse();
    const int halfpatch_size = patch_size/2;
    bool converged=false;
    Eigen::Matrix3f H; H.setZero();
    for(int l = l1; l <= l2; l++){
      pyr.levelTranformCoordinates(cInit,c_level,0,l);
      if(mp.isValidPatch_[l] && mp.patches_[l].isPatchInFrame(pyr.imgs_[l],c_level.get_c(),W,false)){
        mp.patches_[l].computeGradientParameters();
        H(0,0) += pow(0.25,l)*mp.patches_[l].H_(0,0);
        H(0,1) += pow(0.25,l)*mp.patches_[l].H_(0,1);
        H(1,0) += pow(0.25,l)*mp.patches_[l].H_(1,0);
        H(1,1) += pow(0.25,l)*mp.patches_[l].H_(1,1);
        H(2,0) += pow(0.5,l)*mp.patches_[l].H_(2,0);
        H(2,1) += pow(0.5,l)*mp.patches_[l].H_(2,1);
        H(0,2) += pow(0.5,l)*mp.patches_[l].H_(0,2);
        H(1,2) += pow(0.5,l)*mp.patches_[l].H_(1,2);
        H(2,2) += mp.patches_[l].H_(2,2);
      }
    }
    Eigen::Matrix3f Hinv = H.inverse();
    float mean_diff = 0;

    // termination condition
    const float min_update_squared = minPixUpd*minPixUpd;
    cOut = cInit;
    Eigen::Vector3f update; update.setZero();
    for(int iter = 0; iter<maxIter; ++iter){
      if(isnan(cOut.get_c().x) || isnan(cOut.get_c().y)){
        assert(false);
        return false;
      }
      Eigen::Vector3f Jres; Jres.setZero();
      int count = 0;
      for(int l = l1; l <= l2; l++){
        pyr.levelTranformCoordinates(cOut,c_level,0,l);
        if(mp.isValidPatch_[l] && mp.patches_[l].isPatchInFrame(pyr.imgs_[l],c_level.get_c(),W,false)){
          const int refStep = pyr.imgs_[l].step.p[0];

          const float* it_patch = mp.patches_[l].patch_;
          const float* it_dx = mp.patches_[l].dx_;
          const float* it_dy = mp.patches_[l].dy_;
          if((W-Eigen::Matrix2f::Identity()).norm() < 1e-6){
            const int u_r = floor(c_level.get_c().x);
            const int v_r = floor(c_level.get_c().y);
            if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= pyr.imgs_[l].cols-halfpatch_size || v_r >= pyr.imgs_[l].rows-halfpatch_size){ // TODO: check limits
              return false;
            }
            // compute interpolation weights
            const float subpix_x = c_level.get_c().x-u_r;
            const float subpix_y = c_level.get_c().y-v_r;
            const float wTL = (1.0-subpix_x)*(1.0-subpix_y);
            const float wTR = subpix_x * (1.0-subpix_y);
            const float wBL = (1.0-subpix_x)*subpix_y;
            const float wBR = subpix_x * subpix_y;

            const uint8_t* it_img;        // loop through search_patch, interpolate
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
            for(int y=0; y<patch_size; ++y){
              for(int x=0; x<patch_size; ++x, ++it_patch, ++it_dx, ++it_dy){
                const float dx = x - halfpatch_size + 0.5;
                const float dy = y - halfpatch_size + 0.5;
                const float wdx = W(0,0)*dx + W(0,1)*dy;
                const float wdy = W(1,0)*dx + W(1,1)*dy;
                const float u_pixel = c_level.get_c().x + wdx - 0.5;
                const float v_pixel = c_level.get_c().y + wdy - 0.5;
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
      if((W-Eigen::Matrix2f::Identity()).norm() < 1e-6){
        update = Hinv * Jres;
      } else {
        update = aff * Hinv * Jres;
      }
      cOut.set_c(cv::Point2f(cOut.get_c().x + update[0],cOut.get_c().y + update[1]));
      mean_diff += update[2];

      if(update[0]*update[0]+update[1]*update[1] < min_update_squared){
        converged=true;
        break;
      }
    }
    return converged;
  }

  /** \brief 2D patch alignment. No guarantee that final coordinates are fully in the frame.
   *
   * @param cOut        - Estimated coordinates for the patch alignment.
   * @param pyr         - Considered image pyramid.
   * @param mp          - \ref MultilevelPatch, which contains the patches.
   * @param cInit       - Coordinates of the patch in the reference image, initial guess.
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param l1          - Start pyramid level (l1<l2)
   * @param l2          - End pyramid level (l1<l2)
   * @param maxIter     - Maximal number of iterations
   * @param minPixUpd   - Termination condition on absolute pixel update
   * @return true, if alignment converged!
   */
  bool align2D(FeatureCoordinates& cOut, const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& cInit, const FeatureWarping* mpWarp,
               const int l1, const int l2, const int maxIter = 10, const double minPixUpd = 0.03){
    // termination condition
    const float min_update_squared = minPixUpd*minPixUpd;
    cOut = cInit;
    Eigen::Vector2f update;
    update.setZero();
    bool converged = false;
    for(int iter = 0; iter<maxIter; ++iter){
      if(isnan(cOut.get_c().x) || isnan(cOut.get_c().y)){
        assert(false);
        return false;
      }
      if(!getLinearAlignEquations(pyr,mp,cOut,mpWarp,l1,l2,A_,b_)){
        return false;
      }
      svd_.compute(A_, Eigen::ComputeThinU | Eigen::ComputeThinV);
      if(svd_.nonzeroSingularValues()<2){
        return false;
      }
      update = svd_.solve(b_);
      cOut.set_c(cv::Point2f(cOut.get_c().x + update[0],cOut.get_c().y + update[1]));

      if(update[0]*update[0]+update[1]*update[1] < min_update_squared){
        converged=true;
        break;
      }
    }
    return converged;
  }

  /** \brief Execute a 2D patch alignment using only one single pyramid level (patch) of the MultilevelPatchFeature.
   *
   * @param cOut        - Estimated coordinates for the patch alignment.
   * @param pyr         - Considered image pyramid.
   * @param mp          - \ref MultilevelPatch, which contains the patches.
   * @param cInit       - Coordinates of the patch in the reference image, initial guess.
   * @param mpWarp      - Affine warping matrix. If nullptr not warping is considered.
   * @param l           - Pyramid level which is used for the alignement
   * @return true, if alignment converged!
   */
  bool align2DSingleLevel(FeatureCoordinates& cOut, const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& cInit, const FeatureWarping* mpWarp, const int l){
    return align2D(cOut,pyr,mp,cInit,mpWarp,l,l);
  }

  /** \brief Aligns a MultilevelPatchFeature to a given image pyramid, coarse to fine
   *
   * @param cOut          - Estimated coordinates for the patch alignment.
   * @param pyr           - Considered image pyramid.
   * @param mp            - \ref MultilevelPatch, which contains the patches.
   * @param cInit         - Coordinates of the patch in the reference image, initial guess.
   * @param mpWarp        - Affine warping matrix. If nullptr not warping is considered.
   * @param lowest_level  - Lowest pyramid level to be considered
   * @param highest_level - Highest pyramid level to be considered (should be smaller than lowest_level)
   * @param start_level   - Start pyramid level for the coarse to fine alignment  (should be SEQ than lowest_level and LEQ than highest_level)
   * @return true, if alignment converged!
   */
  bool align2DComposed(FeatureCoordinates& cOut, const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& cInit, const FeatureWarping* mpWarp,
                       const int lowest_level,const int highest_level, const int start_level){
    cOut = cInit;
    for(int l = start_level;l>=highest_level;--l){
      if(!align2D(cOut,pyr,mp,cOut,mpWarp,l,lowest_level)){
        return false;
      }
    }
    return true;
  }

  /** \brief Aligns a MultilevelPatchFeature to a given image pyramid, adapts the algorithm to the uncertainty of cInit
   *
   * @param cOut          - Estimated coordinates for the patch alignment.
   * @param pyr           - Considered image pyramid.
   * @param mp            - \ref MultilevelPatch, which contains the patches.
   * @param cInit         - Coordinates of the patch in the reference image, initial guess.
   * @param mpWarp        - Affine warping matrix. If nullptr not warping is considered.
   * @param lowest_level  - Lowest pyramid level to be considered
   * @param highest_level - Highest pyramid level to be considered (should be smaller than lowest_level)
   * @param convergencePixelRange - what is the expected converges range (one-sided, gets scaled by the patch level), 1 is a good value
   * @param coverageRatio - How much of the uncertainty should be covered, 2 is a good value
   * @param maxUniSample  - How many samples should maximally be evaluated, one-sided
   * @return true, if alignment converged!
   */
  bool align2DAdaptive(FeatureCoordinates& cOut, const ImagePyramid<nLevels>& pyr, const MultilevelPatch<nLevels,patch_size>& mp, const FeatureCoordinates& cInit, const FeatureWarping* mpWarp,
                       const int lowest_level = nLevels,const int highest_level = 0, const double convergencePixelRange = 1.0,  const double coverageRatio = 2.0, const int maxUniSample = 5){
    bestIntensityError_ = -1;
    cOut = cInit;
    const int n = std::min(std::max(static_cast<int>(ceil((cInit.sigma1_*coverageRatio)/(convergencePixelRange*pow(2.0,lowest_level+1))-0.5)),0),maxUniSample); // (n+0.5)*r*2^(l+1) > s*f
    if(n==0){ // Catch simple case
      return align2D(cOut,pyr,mp,cInit,mpWarp,highest_level,lowest_level);
    }
    for(int i = -n;i<=n;i++){ // i is the multiple of steps which should be taken along the directions
      cOut.set_c(cInit.get_c() + vecToPoint2f(cInit.eigenVector1_.cast<float>()*i*convergencePixelRange*pow(2.0,lowest_level+1)));
      if(align2D(cOut,pyr,mp,cOut,mpWarp,highest_level,lowest_level)){
        if(mlpTemp_.isMultilevelPatchInFrame(pyr,cOut,lowest_level,mpWarp,false)){
          mlpTemp_.extractMultilevelPatchFromImage(pyr,cOut,lowest_level,mpWarp,false);
          const float avgError = mlpTemp_.computeAverageDifference(mp,highest_level,lowest_level);
          if(bestIntensityError_ == -1 || avgError<bestIntensityError_){
            bestCoordinateMatch_ = cOut;
            bestIntensityError_ = avgError;
          }
        }
      }
    }
    if(bestIntensityError_ == -1){
      return false;
    } else {
      cOut = bestCoordinateMatch_;
      return true;
    }
  }
};

}


#endif /* ROVIO_MULTILEVELPATCHALIGNMENT_HPP_ */
