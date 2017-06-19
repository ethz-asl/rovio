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

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/State.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rovio/Camera.hpp"
#include "rovio/FeatureDistance.hpp"

namespace rovio{

/** \brief Helper function for transforming a cv::Point2f to a Eigen::Vector2f
 *
 * @param p - cv::Point2f
 * @return Eigen::Vector2f
 */
static Eigen::Vector2f pointToVec2f(const cv::Point2f& p){
  return Eigen::Vector2f(p.x,p.y);
}


/** \brief Helper function for transforming a cv::Point2f to a Eigen::Vector2f
 *
 * @param v - Eigen::Vector2f
 * @return cv::Point2f
 */
static cv::Point2f vecToPoint2f(const Eigen::Vector2f& v){
  return cv::Point2f(v(0),v(1));
}

/** \brief FeatureCoordinates class, contains information about the location of the feature.
 *
 * Automatically transforms between pixel coordinates and bearing vector (has an internal pointer on a camera).
 * Furthermore it also stores the cam ID, and handles uncertainties associated with the feature location.
 * It also handles local patch warping.
 */
class FeatureCoordinates{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  mutable cv::Point2f c_;  /**<Pixel coordinates of the feature.*/
  mutable bool valid_c_;  /**<Bool, indicating if the current feature pixel coordinates \ref c_ are valid.*/
  mutable LWF::NormalVectorElement nor_;  /**<Bearing vector, belonging to the feature.*/
  mutable bool valid_nor_;  /**<Bool, indicating if the current bearing vector \ref nor_ is valid.*/
  const Camera* mpCamera_;  /**<Pointer to the associated camera object.*/
  Eigen::Matrix2d pixelCov_;  /**<Estimated covariance of pixel coordinates.*/
  Eigen::Vector2d eigenVector1_;  /**<Estimated eigenvector of major uncertainty axis.*/
  Eigen::Vector2d eigenVector2_;  /**<Estimated eigenvector of minor uncertainty axis.*/
  double sigma1_;  /**<Standard deviation in the direction of the major axis of the uncertainty ellipse belonging to \ref c_.*/
  double sigma2_;  /**<Standard deviation in the direction of the semi-major axis of the uncertainty ellipse belonging to \ref c_.*/
  double sigmaAngle_; /**<Angle between the x-axis and the major axis of the uncertainty ellipse belonging to \ref c_.*/
  mutable Eigen::EigenSolver<Eigen::Matrix2d> es_; /**<Solver for computing the geometry of the pixel uncertainty.*/
  int camID_; /**<Camera ID.*/

  mutable Eigen::Matrix2f warp_c_;  /**<Transformation matrix for pixel coordinates (from current patch to the current frame).*/
  mutable bool valid_warp_c_;  /**<Specifies if the transformation \ref warp_c_ is valid.*/
  mutable Eigen::Matrix2d warp_nor_;  /**<Transformation matrix for bearing vector (from current patch to the current frame).*/
  mutable bool valid_warp_nor_;  /**<Specifies if the transformation \ref warp_nor_ is valid.*/
  bool isWarpIdentity_; /**<Is the warping equal to identity (on pixel level, warp_c_).*/
  bool trackWarping_; /**<Should the warping be tracked.*/
  mutable Eigen::Matrix2d matrix2dTemp_; /**<Temporary Matrix2d.*/
  mutable Eigen::FullPivLU<Eigen::Matrix2d> fullPivLU2d_; /**<Full pivot LU decomposition for rank and inverse computation of Matrix2d.*/
  mutable LWF::NormalVectorElement norTemp_; /**<Temporary normal vector.*/

  /** \brief Constructor
   */
  FeatureCoordinates();

  /** \brief Constructor based on given pixel coordinates
   *
   *  @param pixel - the pixel coordinates the feature should be set to.
   */
  FeatureCoordinates(const cv::Point2f& pixel);

  /** \brief Constructor based on given pixel coordinates
   *
   *  @param nor - the bearing vector the feature should be set to.
   */
  FeatureCoordinates(const LWF::NormalVectorElement& nor);

  /** \brief Constructor
   */
  FeatureCoordinates(const Camera* mpCamera);

  /** \brief Destructor
   */
  virtual ~FeatureCoordinates();

  /** \brief Resets the feature coordinates \ref c_, \ref nor_.
   *
   *  Note: that the values of the feature coordinates \ref c_ and \ref nor_ are not deleted. They are just set invalid.
   */
  void resetCoordinates();

  /** \brief Compute the feature's pixel coordinates \ref c_. Typically used in conjunction with \ref get_c.
   *
   *   If there are no valid feature pixel coordinates \ref c_ available, the feature pixel coordinates are computed
   *   from the bearing vector \ref nor_ (if valid).
   *  @return whether the computation was successful
   */
  bool com_c() const;

  /** \brief Get the feature's pixel coordinates \ref c_. Typically used in conjunction with \ref com_c.
   *
   *  @return the valid feature pixel coordinates \ref c_.
   */
  const cv::Point2f& get_c() const;

  /** \brief Compute the feature's bearing vector \ref nor_. Typically used in conjunction with \ref get_nor.
   *
   *  If there is no valid bearing vector \ref nor_ available, the bearing vector is computed from the
   *  feature pixel coordinates \ref c_ (if valid).
   *  @return the valid bearing vector \ref nor_.
   */
  bool com_nor() const;

  /** \brief Get the feature's bearing vector \ref nor_. Typically used in conjunction with \ref com_nor.
   *
   *  @return the valid bearing vector \ref nor_.
   */
  const LWF::NormalVectorElement& get_nor() const;

  /** \brief Get the feature's pixel coordinates Jacobian.
   *
   *  Note: Validity can be checked with \ref com_c
   *  @return The Jacobian of the pixel output w.r.t. the bearing vector
   */
  Eigen::Matrix<double,2,2> get_J() const;

  /** \brief Sets the feature pixel coordinates \ref c_ and declares them valid (\ref valid_c_ = true).
   *
   *  Note: The bearing vector nor_ is set invalid (valid_nor_ = false).
   *  \see set_nor()
   *  @param c - Feature pixel coordinates.
   *  @param resetWarp - Should the warping be invalidated.
   */
  void set_c(const cv::Point2f& c, const bool resetWarp = true);

  /** \brief Sets the feature's bearing vector \ref nor_ and declares it valid (\ref valid_nor_ = true).
   *
   *  Note: The feature pixel coordinates are set invalid (\ref valid_c_ = false).
   *  \see set_c()
   *  @param nor - Bearing vector.
   *  @param resetWarp - Should the warping be invalidated.
   */
  void set_nor(const LWF::NormalVectorElement& nor, const bool resetWarp = true);

  /** \brief Compute the pixel warping. If necessary derives it from the bearing warping.
   *
   * @return Whether the computation was successfull.
   */
  bool com_warp_c() const;

  /** \brief Get the pixel warping. If necessary derives it from the bearing warping.
   *
   * @return The valid warping matrix for pixel coordinates.
   */
  Eigen::Matrix2f& get_warp_c() const;

  /** \brief Compute the bearing vector warping. If necessary derives it from the pixel warping.
   *
   * @return Whether the computation was successfull.
   */
  bool com_warp_nor() const;

  /** \brief Get the bearing vector warping. If necessary derives it from the pixel warping.
   *
   * @return The valid warping matrix for pixel coordinates.
   */
  Eigen::Matrix2d& get_warp_nor() const;

  /** \brief Returns a desired corner coordinate (in the patch) as FeatureCoordinate
   *
   * Note: Validity can be checked with com_warp_nor
   * @param x - x coordinate of corner
   * @param y - y coordinate of corner
   * @return the corner coordinate
   */
  FeatureCoordinates get_patchCorner(const double x, const double y) const;

  /** \brief Set the warping matrix for pixel coordinates \ref warp_c_.
   *
   * Note: The validity of \ref warp_nor_ is set to invalid.
   * @param warp_c - warping matrix in pixel coordinates.
   */
  void set_warp_c(const Eigen::Matrix2f& warp_c);

  /** \brief Set the warping matrix for bearing coordinates \ref warp_nor_.
   *
   * Note: The validity of \ref warp_c_ is set to invalid.
   * @param warp_nor - warping matrix for bearing vector.
   */
  void set_warp_nor(const Eigen::Matrix2d& warp_nor);

  /** \brief Set the warping to identity (pixel coordinates).
   *
   * Note: The validity of \ref warp_nor_ is set to invalid.
   */
  void set_warp_identity();

  /** \brief Checks if the feature coordinates can be associated with a landmark in front of the camera.
   *
   *   @return true, if the feature coordinates can be associated with a landmark in front of the camera.
   */
  bool isInFront() const;

  /** \brief Checks if warping is near identity for pixel coordinates
   *
   *   @return true, if the feature warping is near identity
   */
  bool isNearIdentityWarping() const;

  /** \brief Sets the feature coordinates standard deviation values \ref pixelCov_, \ref sigma1_, \ref sigma2_, \ref eigenVector1_, \ref eigenVector2_, \ref sigmaAngle_
   *         from a 2x2 covariance matrix.
   *
   *  @param cov - Covariance matrix (2x2).
   */
  void setPixelCov(const Eigen::Matrix2d& cov);

  /** \brief Draws a point at given feature coordinates.
   *
   *  @param drawImg - Image in which the point should be drawn.
   *  @param color   - Color of the point.
   */
  void drawPoint(cv::Mat& drawImg, const cv::Scalar& color, const float s = 2) const;

  /** \brief Draws an uncertainty ellipse at given feature coordinates.
   *
   *  @param drawImg         - Image in which the uncertainty ellipse should be drawn.
   *  @param color           - Color of the ellipse.
   *  @param scaleFactor     - Scale Factor of the uncertainty ellipse. If set to 1, the ellipse axes lengths match the true standard deviation values.
   *  @param withCenterPoint - Set to true if the center point of the ellipse should be drawn.
   */
  void drawEllipse(cv::Mat& drawImg, const cv::Scalar& color, double scaleFactor = 2.0, const bool withCenterPoint = true) const;

  /** \brief Draws a line between two feature coordinates.
   *
   *  @param drawImg     - Image in which the line should be drawn.
   *  @param other       - Feature coordinates to draw a line to.
   *  @param color       - Color of the line.
   *  @param thickness   - Thickness of the line.
   */
  void drawLine(cv::Mat& drawImg, const FeatureCoordinates& other, const cv::Scalar& color, int thickness = 2) const;

  /** \brief Draws a text at specific feature coordinates.
   *
   *  @param drawImg     - Image in which the text should be drawn.
   *  @param s           - Text.
   *  @param color       - Color of the text.
   */
  void drawText(cv::Mat& drawImg, const std::string& s, const cv::Scalar& color) const;

  /** \brief Get the depth value from the triangulation of two bearing vectors (this is in C1, other is in C2).
   *
   *  @param other   - Bearing vector in another frame C2 (unit length!).
   *  @param C2rC2C1 - Position vector, pointing from C2 to C1, expressed in coordinates of C2.
   *  @param qC2C1   - Quaternion, expressing the orientation of C1 in the C2.
   *  @param d       - Triangulated depth value along the bearing vector C1fP.
   *  @param minDistance - Minimal distance being considered.
   *  @return true, if triangulation successful. This means the angle between the projection rays has not been too small.
   */
  bool getDepthFromTriangulation(const FeatureCoordinates& other, const V3D& C2rC2C1, const QPD& qC2C1, FeatureDistance& d, const double minDistance = 0);

  /** \brief Get the depth uncertainty tau of a triangulated depth value (this is in C1, other is in C2).
   *
   *  Consider a bearing vector C1fP in a reference frame and a bearing vector C2fP in a partner frame have been used
   *  to triangulate a depth value d (along the bearing vector C1fP). Let's call the so gained 3D landmark position P.
   *  In order to get depth uncertainty value of d (along the bearing vector C1fP), a constant pixel error
   *  of the detection of C2fP can be projected to the ray of C1fP (to the side which is farther to the reference frame).
   *  Let's call 3D point corresponding to the maximal pixel error P_plus.
   *  The depth uncertainty tau is then defined as \f$tau=|(P\_plus - P)|\f$.
   *
   *  @param C1rC1C2        - Position vector, pointing from C1 to C2, expressed in coordinates of C1.
   *  @param d              - Triangulated depth value along the bearing vector C1fP.
   *  @param px_error_angle - Angle between the bearing vector C2fP and the bearing vector corresponding to the maximal
   *                          pixel error. <br>
   *                          Compute it as: <br>
   *                           \f$px\_error\_angle = 2 \cdot \arctan{\frac{px\_error}{2 \cdot fx}}\f$ <br>
   *                          ,where px_error is the assumed pixel error (e.g. 1 pixel) and
   *                          fx the focal length (expressed in pixel) of the camera belonging to C2fP.
   * @return the depth uncertainty value \f$tau=|(P\_plus - P)|\f$.
   *
   * @todo rethink
   */
  float getDepthUncertaintyTau(const V3D& C1rC1C2, const float d, const float px_error_angle);
};

// Convenience type to help deal with alignment issues
// https://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
using FeatureCoordinatesVec = std::vector<FeatureCoordinates, Eigen::aligned_allocator<FeatureCoordinates>>;

}


#endif /* FEATURECOORDINATES_HPP_ */
