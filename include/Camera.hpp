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

#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include "State.hpp"

using namespace Eigen;

namespace rovio{

class Camera{
 public:
  Matrix3d K_;
  int h_,w_;
  double k1_,k2_,k3_,k4_,k5_,k6_;
  double p1_,p2_,s1_,s2_,s3_,s4_;
  Camera(){
    k1_ = 0.0; k2_ = 0.0; k3_ = 0.0; k4_ = 0.0; k5_ = 0.0; k6_ = 0.0;
    p1_ = 0.0; p2_ = 0.0; s1_ = 0.0; s2_ = 0.0; s3_ = 0.0; s4_ = 0.0;
    h_ = 480; w_ = 640;
    K_.setIdentity();
  };
  ~Camera(){};
  void distort(const Eigen::Vector2d& in, Eigen::Vector2d& out){
    const double x2 = in(0) * in(0);
    const double y2 = in(1) * in(1);
    const double xy = in(0) * in(1);
    const double r2 = x2 + y2;
    const double kr = (1 + ((k3_ * r2 + k2_) * r2 + k1_) * r2);
    out(0) = in(0) * kr + p1_ * 2 * xy + p2_ * (r2 + 2 * x2);
    out(1) = in(1) * kr + p1_ * (r2 + 2 * y2) + p2_ * 2 * xy;
  }
  void distort(const Eigen::Vector2d& in, Eigen::Vector2d& out, Eigen::Matrix2d& J){
    const double x2 = in(0) * in(0);
    const double y2 = in(1) * in(1);
    const double xy = in(0) * in(1);
    const double r2 = x2 + y2;
    const double kr = (1 + ((k3_ * r2 + k2_) * r2 + k1_) * r2);
    out(0) = in(0) * kr + p1_ * 2 * xy + p2_ * (r2 + 2 * x2);
    out(1) = in(1) * kr + p1_ * (r2 + 2 * y2) + p2_ * 2 * xy;
    J(0,0) = kr + 2.0 * k1_ * x2 + 4.0 * k2_ * x2 * r2 + 6.0 * k3_ * x2 * r2 * r2 + 2.0 * p1_ * in(1) + 6.0 * p2_ * in(0);
    J(0,1) = 2.0 * k1_ * xy + 4.0 * k2_ * xy * r2 + 6.0 * k3_ * xy * r2 * r2 + 2 * p1_ * in(0) + 2 * p2_ * in(1);
    J(1,0) = J(0,1);
    J(1,1) = kr + 2.0 * k1_ * y2 + 4.0 * k2_ * y2 * r2 + 6.0 * k3_ * y2 * r2 * r2 + 6.0 * p1_ * in(1) + 2.0 * p2_ * in(0);
  }
  bool bearingToPixel(const Eigen::Vector3d& vec, cv::Point2f& c){
    // Project
    if(vec(2)<=0) return false;
    const Eigen::Vector2d undistorted = Eigen::Vector2d(vec(0)/vec(2),vec(1)/vec(2));

    // Distort
    Eigen::Vector2d distorted;
    distort(undistorted,distorted);

    // Shift origin and scale
    c.x = static_cast<float>(K_(0, 0)*distorted(0) + K_(0, 2));
    c.y = static_cast<float>(K_(1, 1)*distorted(1) + K_(1, 2));
    if(c.x < 0 || c.y < 0 || c.x >= w_ || c.y >= h_){
      return false;
    }
    return true;
  }
  bool bearingToPixel(const Eigen::Vector3d& vec, cv::Point2f& c, Eigen::Matrix<double,2,3>& J){
    // Project
    if(vec(2)<=0) return false;
    const Eigen::Vector2d undistorted = Eigen::Vector2d(vec(0)/vec(2),vec(1)/vec(2));
    Eigen::Matrix<double,2,3> J1; J1.setZero();
    J1(0,0) = 1.0/vec(2);
    J1(0,2) = -vec(0)/pow(vec(2),2);
    J1(1,1) = 1.0/vec(2);
    J1(1,2) = -vec(1)/pow(vec(2),2);

    // Distort
    Eigen::Vector2d distorted;
    Eigen::Matrix2d J2;
    distort(undistorted,distorted,J2);

    // Shift origin and scale
    c.x = static_cast<float>(K_(0, 0)*distorted(0) + K_(0, 2));
    c.y = static_cast<float>(K_(1, 1)*distorted(1) + K_(1, 2));
    Eigen::Matrix2d J3; J3.setZero();
    J3(0,0) = K_(0, 0);
    J3(1,1) = K_(1, 1);

    J = J3*J2*J1;

    if(c.x < 0 || c.y < 0 || c.x >= w_ || c.y >= h_){
      return false;
    }
    return true;
  }
  bool bearingToPixel(const LWF::NormalVectorElement& vec, cv::Point2f& c){
    return bearingToPixel(vec.n_,c);
  }
  bool bearingToPixel(const LWF::NormalVectorElement& vec, cv::Point2f& c, Eigen::Matrix<double,2,2>& J){
    Eigen::Matrix<double,3,2> J1;
    J1 = vec.getM();
    Eigen::Matrix<double,2,3> J2;
    const bool success = bearingToPixel(vec.n_,c,J2);
    J = J2*J1;
    return success;
  }
  bool pixelToBearing(const cv::Point2f& c,Eigen::Vector3d& vec){
    // Shift origin and scale
    Eigen::Vector2d y;
    y(0) = (static_cast<double>(c.x) - K_(0, 2)) / K_(0, 0);
    y(1) = (static_cast<double>(c.y) - K_(1, 2)) / K_(1, 1);

    // Undistort by optimizing
    const int max_iter = 5;
    const double tolerance = 1e-15;
    Eigen::Vector2d ybar = y;
    Eigen::Matrix2d J;
    Eigen::Vector2d y_tmp;
    Eigen::Vector2d e;
    Eigen::Vector2d du;
    for (int i = 0; i < max_iter; i++) {
      y_tmp = ybar;
      distort(ybar,y_tmp,J);
      e = y - y_tmp;
      du = (J.transpose() * J).inverse() * J.transpose() * e;
      ybar += du;
      if (e.dot(e) <= tolerance) break;
    }
    y = ybar;
    vec = Eigen::Vector3d(y(0),y(1),1.0).normalized();
    return true;
  }
  bool pixelToBearing(const cv::Point2f& c,LWF::NormalVectorElement& vec){
    return pixelToBearing(c,vec.n_);
  }
  void testCameraModel(){
    double d = 1e-4;
    LWF::NormalVectorElement b_s;
    LWF::NormalVectorElement b_s1;
    LWF::NormalVectorElement b_s2;
    Eigen::Vector3d v_s;
    Eigen::Vector3d v_s1;
    Eigen::Vector3d v_s2;
    LWF::NormalVectorElement b_e;
    Eigen::Matrix2d J1;
    Eigen::Matrix2d J1_FD;
    Eigen::Matrix<double,2,3> J2;
    Eigen::Matrix<double,2,3> J2_FD;
    cv::Point2f p_s;
    cv::Point2f p_s1;
    cv::Point2f p_s2;
    cv::Point2f p_s3;
    Eigen::Vector2d diff;
    for(unsigned int s = 1; s<10;){
      b_s.setRandom(s);
      if(b_s.n_(2)<0) b_s.n_(2) = -b_s.n_(2);
      bearingToPixel(b_s,p_s,J1);
      pixelToBearing(p_s,b_e);
      b_s.boxMinus(b_e,diff);
      std::cout << b_s.n_.transpose() << std::endl;
      std::cout << "Error after back and forward mapping: " << diff.norm() << std::endl;
      diff = Eigen::Vector2d(d,0);
      b_s.boxPlus(diff,b_s1);
      bearingToPixel(b_s1,p_s1);
      J1_FD(0,0) = static_cast<double>((p_s1-p_s).x)/d;
      J1_FD(1,0) = static_cast<double>((p_s1-p_s).y)/d;
      diff = Eigen::Vector2d(0,d);
      b_s.boxPlus(diff,b_s2);
      bearingToPixel(b_s2,p_s2);
      J1_FD(0,1) = static_cast<double>((p_s2-p_s).x)/d;
      J1_FD(1,1) = static_cast<double>((p_s2-p_s).y)/d;
      std::cout << J1 << std::endl;
      std::cout << J1_FD << std::endl;

      v_s = b_s.n_;
      bearingToPixel(v_s,p_s,J2);
      bearingToPixel(v_s + Eigen::Vector3d(d,0,0),p_s1);
      bearingToPixel(v_s + Eigen::Vector3d(0,d,0),p_s2);
      bearingToPixel(v_s + Eigen::Vector3d(0,0,d),p_s3);
      J2_FD(0,0) = static_cast<double>((p_s1-p_s).x)/d;
      J2_FD(1,0) = static_cast<double>((p_s1-p_s).y)/d;
      J2_FD(0,1) = static_cast<double>((p_s2-p_s).x)/d;
      J2_FD(1,1) = static_cast<double>((p_s2-p_s).y)/d;
      J2_FD(0,2) = static_cast<double>((p_s3-p_s).x)/d;
      J2_FD(1,2) = static_cast<double>((p_s3-p_s).y)/d;
      std::cout << J2 << std::endl;
      std::cout << J2_FD << std::endl;
    }
  }
  //  void pixelToBearingVector(const cv::Point2f& c,Eigen::Vector3d& v){ // radtan (extended)
  //    // Shift origin and scale for undistortion
  //    double x = (c.x - K_(0, 2)) / K_(0, 0);
  //    double y = (c.y - K_(1, 2)) / K_(1, 1);
  //    // Undistortion // TODO this is distortion!!!!!!!!!!!!!!!
  //    double x2 = x * x;
  //    double y2 = y * y;
  //    double r2 = x2 + y2;
  //    double kr = (1 + ((k3_ * r2 + k2_) * r2 + k1_) * r2) / (1 + ((k6_ * r2 + k5_) * r2 + k4_) * r2);
  //    double xd = x * kr + p1_ * 2 * x * y + p2_ * (r2 + 2 * x2) + s1_ * r2
  //    + s2_ * r2 * r2;
  //    double yd = y * kr + p1_ * (r2 + 2 * y2) + p2_ * 2 * x * y + s3_ * r2
  //    + s4_ * r2 * r2;
  //    v = Eigen::Vector3d(xd,yd,1.0).normalized();
  //  }
};

}


#endif /* CAMERA_HPP_ */
