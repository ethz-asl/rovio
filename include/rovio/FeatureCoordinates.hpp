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

#ifndef FEATURECOORDINATES_HPP_
#define FEATURECOORDINATES_HPP_

#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include "lightweight_filtering/State.hpp"
#include "rovio/Camera.hpp"

namespace rovio{

class FeatureCoordinates{
 public:
  mutable cv::Point2f c_;  /**<Pixel coordinates of the feature.*/
  mutable bool valid_c_;  /**<Bool, indicating if the current feature pixel coordinates \ref c_ are valid.*/
  mutable LWF::NormalVectorElement nor_;  /**<Bearing vector, belonging to the feature.*/
  mutable bool valid_nor_;  /**<Bool, indicating if the current bearing vector \ref nor_ is valid.*/
  const Camera* mpCamera_;  /**<Pointer to the associated camera object.*/
  double sigma1_;  /**<Standard deviation in the direction of the major axis of the uncertainty ellipse belonging to \ref c_.*/
  double sigma2_;  /**<Standard deviation in the direction of the semi-major axis of the uncertainty ellipse belonging to \ref c_.*/
  double sigmaAngle_; /**<Angle between the x-axis and the major axis of the uncertainty ellipse belonging to \ref c_.*/
  Eigen::EigenSolver<Eigen::Matrix2d> es_; /**<Solver for computing the geometry of the pixel uncertainty.*/

  /** \brief Constructor
   */
  FeatureCoordinates(){
    mpCamera_ = nullptr;
    resetCoordinates();
  }

  /** \brief Constructor
   */
  FeatureCoordinates(const Camera* mpCamera): mpCamera_(mpCamera){
    resetCoordinates();
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

  /** \brief Get the feature's pixel coordinates \ref c_.
   *
   *   If there are no valid feature pixel coordinates \ref c_ available, the feature pixel coordinates are computed
   *   from the bearing vector \ref nor_ (if valid).
   *  @return the valid feature pixel coordinates \ref c_.
   */
  const cv::Point2f& get_c() const{
    if(!valid_c_){
      if(mpCamera_ == nullptr) throw cameraNullPtrException;
      if(valid_nor_ && mpCamera_->bearingToPixel(nor_,c_)){
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
      if(mpCamera_ == nullptr) throw cameraNullPtrException;
      if(valid_c_ && mpCamera_->pixelToBearing(c_,nor_)){
        valid_nor_ = true;
      } else {
        std::cout << "ERROR: No valid coordinate data!" << std::endl;
      }
    }
    return nor_;
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
  void setSigmaFromCov(const Eigen::Matrix2d& cov){ // TODO: put somewhere else
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

  /** \brief Draws a point at given feature coordinates.
   *
   *  @param drawImg - Image in which the point should be drawn.
   *  @param color   - Color of the point.
   */
  void drawPoint(cv::Mat& drawImg, const cv::Scalar& color) const{
    cv::Size size(2,2);
    cv::ellipse(drawImg,get_c(),size,0,0,360,color,-1,8,0);
  }

  /** \brief Draws an uncertainty ellipse at given feature coordinates.
   *
   *  @param drawImg         - Image in which the uncertainty ellipse should be drawn.
   *  @param color           - Color of the ellipse.
   *  @param scaleFactor     - Scale Factor of the uncertainty ellipse. If set to 1, the ellipse axes lengths match the true standard deviation values.
   *  @param withCenterPoint - Set to true if the center point of the ellipse should be drawn.
   */
  void drawEllipse(cv::Mat& drawImg, const cv::Scalar& color, double scaleFactor = 2.0, const bool withCenterPoint = true) const{
    if(withCenterPoint) drawPoint(drawImg,color);
    cv::ellipse(drawImg,get_c(),cv::Size(std::max(static_cast<int>(scaleFactor*sigma1_+0.5),1),std::max(static_cast<int>(scaleFactor*sigma2_+0.5),1)),sigmaAngle_*180/M_PI,0,360,color,1,8,0);
  }

  /** \brief Draws a line between two feature coordinates.
   *
   *  @param drawImg     - Image in which the line should be drawn.
   *  @param other       - Feature coordinates to draw a line to.
   *  @param color       - Color of the line.
   *  @param thickness   - Thickness of the line.
   */
  void drawLine(cv::Mat& drawImg, FeatureCoordinates& other, const cv::Scalar& color, int thickness = 2) const{
    cv::line(drawImg,get_c(),other.get_c(),color,thickness);
  }

  /** \brief Draws a text at specific feature coordinates.
   *
   *  @param drawImg     - Image in which the text should be drawn.
   *  @param s           - Text.
   *  @param color       - Color of the text.
   */
  void drawText(cv::Mat& drawImg, const std::string& s, const cv::Scalar& color) const{
    cv::putText(drawImg,s,get_c(),cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
  }
};

}


#endif /* FEATURECOORDINATES_HPP_ */
