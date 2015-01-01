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
#include <stdio.h>
#include <math.h>
#include <binary-serialization.h>
#include <ros-camera.h>
#include <camera_calibration_parsers/parse_yml.h>
#include <image_transport/image_transport.h>

#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
struct OptimizationFunctor
{
  /** undocumented */
  typedef _Scalar Scalar;
  /** undocumented */
  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  /** undocumented */
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  /** undocumented */
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  /** undocumented */
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  /** undocumented */
  const int m_inputs;
  /** undocumented */
  const int m_values;

  /** undocumented */
  OptimizationFunctor() :
      m_inputs(InputsAtCompileTime),
      m_values(ValuesAtCompileTime) {}
  /** undocumented */
  OptimizationFunctor(int inputs, int values) :
      m_inputs(inputs),
      m_values(values) {}

  /** undocumented */
  int inputs() const
  {
    return m_inputs;
  }
  /** undocumented */
  int values() const
  {
    return m_values;
  }
};

struct undistort_functor : OptimizationFunctor<double> {
  Eigen::Matrix<double, 2, 1> distCoordinates;
  Eigen::Matrix<double, 5, 1> distCoefficients;

  undistort_functor(const Eigen::Matrix<double, 2, 1> & distCoordinates,
                    const Eigen::Matrix<double, 5, 1> & distCoefficients)
      : OptimizationFunctor<double>(2, 2),
        distCoordinates(distCoordinates),
        distCoefficients(distCoefficients) {
  }

  int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
    assert(x.size() == 2);
    assert(fvec.size() == 2);

    double x_prime = x[0];
    double y_prime = x[1];
    double x_prime_y_prime = x_prime * y_prime;
    double x_prime_squared = x_prime * x_prime;
    double y_prime_squared = y_prime * y_prime;
    double r_squared = x_prime_squared + y_prime_squared;
    double r_quad = r_squared * r_squared;
    fvec[0] = (x_prime
        * (1.0 + distCoefficients(0, 0) * r_squared
            + distCoefficients(1, 0) * r_quad)
        + 2.0 * distCoefficients(2, 0) * x_prime_y_prime
        + distCoefficients(3, 0) * (r_squared + 2.0 * x_prime_squared))
        - distCoordinates[0];
    fvec[1] = (y_prime
        * (1.0 + distCoefficients(0, 0) * r_squared
            + distCoefficients(1, 0) * r_quad)
        + 2.0 * distCoefficients(3, 0) * x_prime_y_prime
        + distCoefficients(2, 0) * (r_squared + 2.0 * y_prime_squared))
        - distCoordinates[1];

    return 0;
  }

  int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) {
    assert(x.size() == 2);
    assert(fjac.rows() == 2);
    assert(fjac.cols() == 2);

    fjac(0, 0) = -1.0 - 3.0 * distCoefficients(0, 0) * x[0] * x[0]
        - distCoefficients(0, 0) * x[1] * x[1]
        - 5.0 * distCoefficients(1, 0) * pow(x[0], 4)
        - 6.0 * distCoefficients(1, 0) * x[0] * x[0] * x[1] * x[1]
        - distCoefficients(1, 0) * pow(x[1], 4)
        - 2.0 * distCoefficients(2, 0) * x[1]
        - 2.0 * distCoefficients(3, 0) * x[0]
        - 4.0 * distCoefficients(3, 0) * x[0];
    fjac(1, 0) = -2.0 * distCoefficients(0, 0) * x[0] * x[1]
        - 4.0 * distCoefficients(1, 0) * pow(x[0], 3) * x[1]
        - 4.0 * distCoefficients(1, 0) * x[0] * pow(x[1], 3)
        - 2.0 * distCoefficients(3, 0) * x[1]
        - 2.0 * distCoefficients(2, 0) * x[0];
    fjac(0, 1) = fjac(1, 0);
    fjac(1, 1) = -1.0 - distCoefficients(0, 0) * x[0] * x[0]
        - 3.0 * distCoefficients(0, 0) * x[1] * x[1]
        - distCoefficients(1, 0) * pow(x[0], 4)
        - 6.0 * distCoefficients(1, 0) * x[0] * x[0] * x[1] * x[1]
        - 5.0 * distCoefficients(1, 0) * pow(x[1], 4)
        - 2.0 * distCoefficients(3, 0) * x[0]
        - 2.0 * distCoefficients(2, 0) * x[1]
        - 4.0 * distCoefficients(2, 0) * x[1];

    return 0;
  }
};

RosCamera::RosCamera(std::string calibFile) {
  sensor_msgs::CameraInfo cameraInfo;
  std::string cameraName = "camera";
  camera_calibration_parsers::readCalibrationYml(calibFile, cameraName,
                                                 cameraInfo);

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++)
      cameraMatrix(r, c) = cameraInfo.K.elems[r * 3 + c];
  }

  for (int r = 0; r < 5; r++)
    distCoeff(r, 0) = cameraInfo.D.at(r);

  //extract image size
  width = cameraInfo.width;
  height = cameraInfo.height;

  //print the extracted camera information to verify
  std::cout << "opened Ros-camera" << std::endl;
  std::cout << "the camera-matrix is" << std::endl;
  std::cout << cameraMatrix << std::endl;
  std::cout << "the distortion coefficients are" << std::endl;
  std::cout << distCoeff << std::endl;

  //create inverse cameraMatrix
  invCameraMatrix = cameraMatrix.inverse();

  //Lookup-table: create matrices for undistorted x and y coordinates
  //undistortedX = new MatrixXd(width,height);
  //undistortedY = new MatrixXd(width,height);
  undistortedX.resize(width * height);
  undistortedY.resize(width * height);

  std::string calibration_lut_file = "./camera_calibration.lut";
  bool load_precomputed_LUT = false;
#ifdef __ARM__
  load_precomputed_LUT = true;
#else
  std::ifstream calibration_lut_stream_exists(calibration_lut_file);
  load_precomputed_LUT = calibration_lut_stream_exists.is_open();
#endif

  if (load_precomputed_LUT) {
    int stored_image_width = -1;
    int stored_image_height = -1;
    Eigen::Matrix<double, 3, 3> stored_inv_camera_matrix;

    std::ifstream calibration_lut_stream(calibration_lut_file);
    if (!calibration_lut_stream.is_open()) {
      std::cout << "Failed to load camera calibration_lut from file: "
          << calibration_lut_file << std::endl;
      exit(1);
    }

    Deserialize(&stored_image_width, &calibration_lut_stream);
    Deserialize(&stored_image_height, &calibration_lut_stream);
    Deserialize(&stored_inv_camera_matrix, &calibration_lut_stream);

    if (stored_image_width != width) {
      std::cout << "Image width from LUT loaded form file: "
          << calibration_lut_file
          << "does not match the current image width: "
          << stored_image_width << " vs. " << width << std::endl;
      exit(1);
    }
    if (stored_image_height != height) {
      std::cout << "Image height from LUT loaded form file: "
          << calibration_lut_file
          << "does not match the current image height: "
          << stored_image_height << " vs. " << height << std::endl;
      exit(1);
    }
    if (!stored_inv_camera_matrix.isApprox(invCameraMatrix, 1e-5)) {
      std::cout << "Inverse camera matrix from LUT loaded form file: "
          << calibration_lut_file
          << "does not match the current camera matrix: "
          << stored_inv_camera_matrix << " vs. " << invCameraMatrix
          << std::endl;
      exit(1);
    }

    Deserialize(&undistortedX, &calibration_lut_stream);
    Deserialize(&undistortedY, &calibration_lut_stream);
  } else {
    //now, for each coordinate, create distorted coordinates and find undistorted ones
    for (int r = 0; r < height; r++) {
      std::cout << "Processing camera LUT: " <<
          static_cast<double>(r) / height * 100. << "%    \r";
      std::cout.flush();
      for (int c = 0; c < width; c++) {
        Eigen::Vector3d imageCoordinates;
        imageCoordinates[0] = (double) c;
        imageCoordinates[1] = (double) r;
        imageCoordinates[2] = 1.0;

        Eigen::Vector3d distortedCoordinatesHom = invCameraMatrix * imageCoordinates;
        Eigen::Vector2d distortedCoordinates;
        distortedCoordinates[0] = distortedCoordinatesHom[0]
            / distortedCoordinatesHom[2];
        distortedCoordinates[1] = distortedCoordinatesHom[1]
            / distortedCoordinatesHom[2];

        undistort_functor functor(distortedCoordinates, distCoeff);
        Eigen::NumericalDiff<undistort_functor> numDiff(functor);
        Eigen::LevenbergMarquardt < Eigen::NumericalDiff<undistort_functor> > lm(numDiff);

        Eigen::VectorXd x(2);
        x[0] = distortedCoordinates[0];
        x[1] = distortedCoordinates[1];

        lm.resetParameters();
        lm.parameters.ftol = 1.E10 * Eigen::NumTraits<double>::epsilon();
        lm.parameters.xtol = 1.E10 * Eigen::NumTraits<double>::epsilon();
        lm.parameters.maxfev = 1000;
        lm.minimize(x);

        //undistortedX(r,c) = x[0];
        //undistortedY(r,c) = x[1];
        undistortedX.at(r * width + c) = x[0];
        undistortedY.at(r * width + c) = x[1];
      }
    }
    std::cout << "Done processing camera LUT" << std::endl;

    std::string calibration_lut_file = "./camera_calibration.lut";
    std::cout << "Storing camera LUT in file: " << calibration_lut_file
        << std::endl;
    std::ofstream calibration_lut_stream(calibration_lut_file);
    if (!calibration_lut_stream.is_open()) {
      std::cout << "Failed to save camera calibration_lut to file: "
          << calibration_lut_file << std::endl;
      return;
    }

    Serialize(width, &calibration_lut_stream);
    Serialize(height, &calibration_lut_stream);
    Serialize(invCameraMatrix, &calibration_lut_stream);
    Serialize(undistortedX, &calibration_lut_stream);
    Serialize(undistortedY, &calibration_lut_stream);
  }
}

RosCamera::~RosCamera() {
}

Eigen::Vector3d RosCamera::camToWorld(double x, double y) {
  Eigen::Vector3d worldCoordinates;

  int c = floor(x + 0.5);
  if (c < 0)
    c = 0;
  if (c > width - 1)
    c = width - 1;
  int r = floor(y + 0.5);
  if (r < 0)
    r = 0;
  if (r > height - 1)
    r = height - 1;
  Eigen::VectorXd x_opt(2);
  x_opt[0] = undistortedX.at(r * width + c);  //undistortedX(r,c);
  x_opt[1] = undistortedY.at(r * width + c);  //undistortedY(r,c);

  Eigen::Vector3d imageCoordinates;
  imageCoordinates[0] = x;
  imageCoordinates[1] = y;
  imageCoordinates[2] = 1.0;

  Eigen::Vector3d distortedCoordinatesHom = invCameraMatrix * imageCoordinates;
  Eigen::Vector2d distortedCoordinates;
  distortedCoordinates[0] = distortedCoordinatesHom[0]
      / distortedCoordinatesHom[2];
  distortedCoordinates[1] = distortedCoordinatesHom[1]
      / distortedCoordinatesHom[2];

  undistort_functor functor(distortedCoordinates, distCoeff);
  //NumericalDiff<undistort_functor> numDiff(functor);
  //LevenbergMarquardt< NumericalDiff<undistort_functor> > lm(numDiff);
  Eigen::LevenbergMarquardt<undistort_functor> lm(functor);

  lm.resetParameters();
  lm.parameters.ftol = 1.E10 * Eigen::NumTraits<double>::epsilon();
  lm.parameters.xtol = 1.E10 * Eigen::NumTraits<double>::epsilon();
  lm.parameters.maxfev = 5;
  lm.minimize(x_opt);

  worldCoordinates[0] = x_opt[0];
  worldCoordinates[1] = x_opt[1];
  worldCoordinates[2] = 1.0;

  worldCoordinates = worldCoordinates / worldCoordinates.norm();

  return worldCoordinates;
}

Eigen::Vector2d RosCamera::worldToCam(Eigen::Vector3d worldCoordinates) {
  Eigen::Vector2d imageCoordinates;

  //ok, enough of shit, we implement our own Eigen version now here:
  double x_prime = worldCoordinates(0, 0) / worldCoordinates(2, 0);
  double y_prime = worldCoordinates(1, 0) / worldCoordinates(2, 0);
  double x_prime_y_prime = x_prime * y_prime;
  double x_prime_squared = x_prime * x_prime;
  double y_prime_squared = y_prime * y_prime;
  double r_squared = x_prime_squared + y_prime_squared;
  double r_quad = r_squared * r_squared;
  Eigen::Vector3d distorted;
  distorted(0, 0) = x_prime
      * (1.0 + distCoeff(0, 0) * r_squared + distCoeff(1, 0) * r_quad)
      + 2.0 * distCoeff(2, 0) * x_prime_y_prime
      + distCoeff(3, 0) * (r_squared + 2.0 * x_prime_squared);
  distorted(1, 0) = y_prime
      * (1.0 + distCoeff(0, 0) * r_squared + distCoeff(1, 0) * r_quad)
      + 2.0 * distCoeff(3, 0) * x_prime_y_prime
      + distCoeff(2, 0) * (r_squared + 2.0 * y_prime_squared);
  distorted(2, 0) = 1.0;
  Eigen::Vector3d imageCoordinatesHom = cameraMatrix * distorted;
  imageCoordinates(0, 0) = imageCoordinatesHom(0, 0)
      / imageCoordinatesHom(2, 0);
  imageCoordinates(1, 0) = imageCoordinatesHom(1, 0)
      / imageCoordinatesHom(2, 0);

  return imageCoordinates;
}
