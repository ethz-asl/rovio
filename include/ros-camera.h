/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef FEATURE_TRACKER_ROS_CAMERA_H_
#define FEATURE_TRACKER_ROS_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <opencv/cv.h>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

class RosCamera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::Matrix3d cameraMatrix;
  Eigen::Matrix<double, 5, 1> distCoeff;
  Eigen::Matrix3d invCameraMatrix;
  std::vector<double> undistortedX;
  std::vector<double> undistortedY;
  int width;
  int height;

  RosCamera(std::string calibFile);
  virtual ~RosCamera();

  Eigen::Vector3d camToWorld(double x, double y);
  Eigen::Vector2d worldToCam(Eigen::Vector3d worldCoordinates);
};
#endif  // FEATURE_TRACKER_ROS_CAMERA_H_
