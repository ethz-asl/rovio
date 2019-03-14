/*
 * PCA.cpp
 *
 *  Created on: Feb 22, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/PCA.h"

namespace Mono_LidarPipeline {
PCA::PCA(const std::shared_ptr<Mono_Lidar::DepthEstimatorParameters>& parameters, const Eigen::MatrixXd& pointCloud)
        : _debug(parameters->pca_debug), _treshold_3_abs_min(parameters->pca_treshold_3_abs_min),
          _treshold_3_2_rel_max(parameters->pca_treshold_3_2_rel_max),
          _treshold_2_1_rel_min(parameters->pca_treshold_2_1_rel_min) {
    CalculatePCA(pointCloud, this->_eigenVectorsValues);

    if (_debug)
        PCA::DebugLog();
}

PCA_Result PCA::getResult() {
    // ascending eigen values
    auto ev1 = _eigenVectorsValues[0](3);
    auto ev2 = _eigenVectorsValues[1](3);
    auto ev3 = _eigenVectorsValues[2](3);

    float planarity = (ev2 - ev1) / ev3;
    float linearity = (ev3 - ev2) / ev3;

    if (planarity < _treshold_2_1_rel_min)
        return PCA_Result::Cubic;

    if (linearity > _treshold_3_2_rel_max)
        return PCA_Result::Linear;

    if (ev3 < _treshold_3_abs_min)
        return PCA_Result::Point;

    return PCA_Result::Plane;
}

void PCA::CalculatePCA(const Eigen::MatrixXd& pointCloud, std::vector<Eigen::Vector4d>& eigenVectorsValues) {
    // Input: PCL is matrix with one 3d point per column
    // Output: fourth compoenent variance along principal component
    // calc mean of PCL
    // The pointcloud mean is used for PCA calculation and also as the anchor point for the normal vector
    // which describes the estimated plane
    this->_pclMean = pointCloud.rowwise().mean();
    Eigen::MatrixXd centered = pointCloud.colwise() - _pclMean;
    Eigen::MatrixXd cov = centered * centered.adjoint();
    assert(cov.rows() == 3 && cov.cols() == 3 && "wrong dimensions of covariance matrix");
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);

    Eigen::Vector4d CurComp;
    for (size_t i = 0; i < 3; ++i) {
        CurComp << eig.eigenvectors().col(i) / eig.eigenvectors().col(i).norm(), eig.eigenvalues()(i);
        eigenVectorsValues.push_back(CurComp);
    }
    assert(eigenVectorsValues[0](3) < eigenVectorsValues[1](3) &&
           "values sorted in descending order, ascending needed");
    this->_planeNormal = eigenVectorsValues[0].head(3);
}

void PCA::DebugLog() {
    auto ev1 = _eigenVectorsValues[2](3);
    auto ev2 = _eigenVectorsValues[1](3);
    auto ev3 = _eigenVectorsValues[0](3);

    std::string resultStr;
    resultToString(PCA::getResult(), resultStr);

    std::cout << "PCA -> " << resultStr << ": " << ev1 << ", " << ev2 << ", " << ev3 << std::endl;

    float anisotropy = (ev1 - ev3) / ev1;
    float planarity = (ev2 - ev3) / ev1;
    float sphericity = ev3 / ev1;
    float linearity = (ev1 - ev2) / ev1;

    std::cout << "anisotropy: " << anisotropy << std::endl;
    std::cout << "planarity: " << planarity << std::endl;
    std::cout << "sphericity: " << sphericity << std::endl;
    std::cout << "linearity: " << linearity << std::endl;
}

void PCA::resultToString(const PCA_Result result, std::string& str) {
    switch (result) {
    case PCA_Result::Cubic:
        str = "Cubic";
        break;
    case PCA_Result::Linear:
        str = "Linear";
        break;
    case PCA_Result::Plane:
        str = "Plane";
        break;
    case PCA_Result::Point:
        str = "Point";
        break;
    default:
        str = "unknown";
    }
}
}
