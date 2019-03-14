/*
 * PCA.h
 *
 *  Created on: Feb 22, 2017
 *      Author: wilczynski
 */

#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include "DepthEstimatorParameters.h"

namespace Mono_LidarPipeline {
enum PCA_Result { Point, Linear, Plane, Cubic };

class PCA {
public:
    PCA(const std::shared_ptr<Mono_Lidar::DepthEstimatorParameters>& parameters, const Eigen::MatrixXd& pointCloud);

    PCA_Result getResult();
    inline Eigen::Vector3d& getPlaneAnchorPoint() {
        return _pclMean;
    }
    inline Eigen::Vector3d& getPlaneNormal() {
        return _planeNormal;
    }
    inline Eigen::Vector4d getEigenVector1() {
        return _eigenVectorsValues[0];
    }
    inline Eigen::Vector4d getEigenVector2() {
        return _eigenVectorsValues[1];
    }
    inline Eigen::Vector4d getEigenVector3() {
        return _eigenVectorsValues[2];
    }

private:
    void CalculatePCA(const Eigen::MatrixXd& pointCloud, std::vector<Eigen::Vector4d>& eigenVectorsValues);

    PCA_Result CheckEigenValueTest(const Eigen::Vector3d& eigenValues,
                                   const Eigen::Matrix3d& eigenVectors,
                                   Eigen::Vector3d& planeNormal);

    void DebugLog();

    void resultToString(const PCA_Result result, std::string& str);

    // Calculation results
    std::vector<Eigen::Vector4d> _eigenVectorsValues;
    Eigen::Vector3d _pclMean;
    Eigen::Vector3d _planeNormal;

    // Tresholds for EigenValues which are ordered as ascending
    bool _debug;
    double _treshold_3_abs_min;   // not a Point condition
    double _treshold_3_2_rel_max; // not linear condition
    double _treshold_2_1_rel_min; // not cubic condition
};
}
