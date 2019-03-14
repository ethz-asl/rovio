/*
 * RansacPlane.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: wilczynski
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <chrono>
#include <math.h>

#include <cv.hpp>

#include "monolidar_fusion/Logger.h"
#include "monolidar_fusion/RansacPlane.h"

namespace Mono_Lidar {
RansacPlane::RansacPlane(const std::shared_ptr<Mono_Lidar::DepthEstimatorParameters>& parameters)
        : _planeDistanceTreshold(parameters->ransac_plane_distance_treshold),
          _planeMaxIteraions(parameters->ransac_plane_max_iterations),
          _doUsePlaneRefinement(parameters->ransac_plane_use_refinement),
          _planeRefinementDistance(parameters->ransac_plane_refinement_treshold),
          _planeProbability(parameters->ransac_plane_probability) {

    is_segmented_ = false;
}

void RansacPlane::CalculateInliersPlane(const Cloud::ConstPtr& pointCloud) {
    Mono_Lidar::Logger::Instance().Log(Mono_Lidar::Logger::MethodStart, "RansacPlane::CalculateInliersPlane Sart: ");

    auto start_time_ransac = std::chrono::steady_clock::now();

    _inliersIndex.clear();

    //    std::cout << "Ransac: pcl size before filter=" << pointCloud->points.size() << std::endl;

    // To speed things up we use only a fraction of the pointcloud
    Cloud::Ptr cloudFiltered(new Cloud);

    // First we cut all points with z values out of our interest
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(pointCloud);
    // The filter field name is set to the z coordinate, and the accepted interval values are set to (-inf;-0.5).
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.2, -1.2);
    pass.filter(*cloudFiltered);

    // Filter points with 0 reflectance.
    pcl::PassThrough<Point> pass_intensity;
    pass_intensity.setInputCloud(cloudFiltered);
    pass_intensity.setFilterFieldName("intensity");
    pass_intensity.setFilterLimits(0.05, FLT_MAX);
    pass_intensity.filter(*cloudFiltered);

    // std::cout << "Ransac: pcl size after filter=" << cloudFiltered->points.size() << std::endl;

    // Now we voxelize the cloud
    // pcl::VoxelGrid<Point> sor;
    // sor.setInputCloud(cloudFiltered);
    // sor.setLeafSize(0.15f, 0.15f, 0.15f);
    // sor.filter(*cloudFiltered);

    // std::cout << "Ransac: pcl size after voxelizatiion=" << cloudFiltered->points.size() << std::endl;


    if (cloudFiltered->points.size() < 3) {
        std::cout << "In RansacPlane: Not enough_points for ransac.!!!!!!!!!!" << std::endl;
        for (const auto& el : pointCloud->points) {
            std::cout << el.x << " " << el.y << " " << el.z << " " << el.intensity << std::endl;
        }
        throw ExceptionPclInvalid();
    }

    // std::cout << "Ransac: pcl size after filter=" << cloudFiltered->points.size() << std::endl;

    // Now do RANSAC
    // SACSegmentation is a wrapper for the commented code and should yield the same results.
    //    pcl::SACSegmentation<Point> seg;
    //    seg.setOptimizeCoefficients(_doUsePlaneRefinement);

    //    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    //    seg.setEpsAngle(M_PI / 12.); // 15°

    //    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    //    seg.setMethodType(pcl::SAC_RANSAC);
    //    seg.setDistanceThreshold(_planeDistanceTreshold);
    //    seg.setInputCloud(cloudFiltered);
    //    seg.setProbability(_planeProbability);
    //    seg.setMaxIterations(_planeMaxIteraions);

    //    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //    seg.segment(*inliers, *coefficients);

    //    std::vector<int> _inliersIndex = inliers->indices;
    //    _modelCoeffs << coefficients->values[0], coefficients->values[1], coefficients->values[2],
    //    coefficients->values[3];

    pcl::SampleConsensusModelPerpendicularPlane<Point>::Ptr model_p(
        new pcl::SampleConsensusModelPerpendicularPlane<Point>(cloudFiltered));

    model_p->setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    //    model_p->setEpsAngle(M_PI / 12.); // 15°
    model_p->setEpsAngle(M_PI / 18.); // 10°

    pcl::RandomSampleConsensus<Point> ransac(model_p);
    //		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    ransac.setDistanceThreshold(_planeDistanceTreshold);
    ransac.setMaxIterations(_planeMaxIteraions);
    ransac.setProbability(_planeProbability);
    ransac.computeModel();
    ransac.getInliers(_inliersIndex);

    Eigen::VectorXf modelCoeffs;
    ransac.getModelCoefficients(modelCoeffs);

    std::cout << "Duration ransac="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_ransac)
                     .count()
              << " ms" << std::endl;

    if (_doUsePlaneRefinement) {
        // Take the inliers and recompute a better solution with a least squares on the inliers of RANSAC
        Eigen::VectorXf modelCoeffsRefined;
        model_p->optimizeModelCoefficients(_inliersIndex, modelCoeffs, modelCoeffsRefined);
        model_p->selectWithinDistance(modelCoeffs, _planeRefinementDistance, _inliersIndex);
        _modelCoeffs = modelCoeffsRefined;
    } else {
        _modelCoeffs = modelCoeffs;
    }

    _pointIsInPlane.clear();

    // write inliers indices into map
    for (const int& index : _inliersIndex) {
        _pointIsInPlane.insert(std::pair<int, bool>(index, true));
    }

    // set flag so we know ransac was applied
    is_segmented_ = true;

    std::cout << "Duration ransac+opti="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_ransac)
                     .count()
              << " ms" << std::endl;


    Mono_Lidar::Logger::Instance().Log(Mono_Lidar::Logger::MethodEnd, "RansacPlane::CalculateInliersPlane.");
}

void GroundPlane::getMatrixFromIndices(const Eigen::MatrixXf& cloudMatrix,
                                       const std::vector<int>& inliers,
                                       Eigen::MatrixXf& matrix) const {
    matrix.resize(3, inliers.size());

    for (int j = 0; j < int(inliers.size()); j++) {
        int indexMap = inliers[j];
        matrix.col(j) = cloudMatrix.col(indexMap);
    }
}

bool GroundPlane::CheckPointInPlane(const int index) const {
    return _pointIsInPlane.count(index);
}

SemanticPlane::SemanticPlane(const cv::Mat& img, Camera cam, std::set<int> groundplane_label, double inlier_threshold)
        : cam_(cam), groundplane_label_(groundplane_label), inlier_threshold_(inlier_threshold) {
    semantic_image_ = std::make_unique<cv::Mat>(img);
}

namespace {

// template <int cols>
// Eigen::Matrix<double, 3, cols> project(const Eigen::Matrix<double, 3, cols>& p_in, const Eigen::Matrix3d& intrinsics)
// {
//    using OutType = Eigen::Matrix<double, 3, cols>;
//    // Project.
//    OutType out = intrinsics * p_in;
//    // Normalize.
//    OutType div;
//    div << out.row(2), out.row(2), out.row(2);
//    out = out.array() * div.array();

//    return out;
//}

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Index = int;
std::map<Index, cv::Point> project(const Cloud::Ptr& pcl, const Eigen::Matrix3d& intrin) {
    std::map<Index, cv::Point> out;
    // Convert cloud to Eigen
    Index ind{0};
    for (const auto& p_in : pcl->points) {
        Eigen::Vector3d p = intrin * Eigen::Vector3d{p_in.x, p_in.y, p_in.z};
        p /= p[2];
        out[ind] = cv::Point(p[0], p[1]);
        ind++;
    }
    return out;
}
}

void SemanticPlane::CalculateInliersPlane(const Cloud::ConstPtr& cloud) {
    // Project points to image.
    Cloud::Ptr transformed_cloud(new Cloud());
    pcl::transformPointCloud(*cloud, *transformed_cloud, cam_.transform_cam_lidar);
    std::map<Index, cv::Point> projected_points = project(transformed_cloud, cam_.getIntrinsics());

    // Remove invalid and non groundplane points.
    auto iter = projected_points.begin();
    for (; iter != projected_points.end();) {
        bool is_on_ground = false;

        bool is_invalid = iter->second.x < 0. || iter->second.x > this->semantic_image_->cols || iter->second.y < 0. ||
                          iter->second.y > this->semantic_image_->rows;
        if (!is_invalid) {
            int label = static_cast<int>(this->semantic_image_->at<uchar>(iter->second));
            if (groundplane_label_.find(label) != groundplane_label_.cend()) {
                is_on_ground = true;
            }
        }

        if (is_on_ground) {
            ++iter;
        } else {
            iter = projected_points.erase(iter);
        }
    }

    // Test if we have enough data.
    if (projected_points.size() < 3) {
        std::cout << "In SemanticPlane: Not enough_points for plane fit.!!!!!!!!!!" << std::endl;
        throw ExceptionPclInvalid();
    }

    // Get indices.
    std::vector<int> inliers_cam;
    inliers_cam.resize(projected_points.size());
    std::transform(
        projected_points.cbegin(), projected_points.cend(), inliers_cam.begin(), [](const auto& a) { return a.first; });

    std::cout << "pcl size=" << cloud->points.size() << " inliers size=" << inliers_cam.size() << std::endl;

    // Compute plane from inlier points.
    pcl::SampleConsensusModelPlane<Point>::Ptr model_p(new pcl::SampleConsensusModelPlane<Point>(cloud));
    Eigen::VectorXf model_coeffs;
    {
        Eigen::VectorXf dummy_model_coeffs(4);
        dummy_model_coeffs << 0., 0., 1., 0.;
        // The implementation of a least squares fit is very strange. It is done in optimize Model, where a prior for
        // the
        // coefficients is needed. However this prior is only used as a return value if the input is wrong and is unused
        // for the optimization.
        model_p->optimizeModelCoefficients(inliers_cam, dummy_model_coeffs, model_coeffs);
    }
    std::cout << "done with opti" << std::endl;
    std::cout << "ModelCoeff=" << model_coeffs.transpose() << std::endl;

    // Refit plane to all points that correspond to this model.
    Eigen::VectorXf model_coeffs_refine;
    std::vector<int> inliers_refine;
    {
        model_p->selectWithinDistance(model_coeffs, inlier_threshold_, inliers_refine);
        model_p->optimizeModelCoefficients(inliers_refine, model_coeffs, model_coeffs_refine);
    }

    std::cout << "SemanticPlane: num_inliers=" << inliers_cam.size() << " num_inliers_refine=" << inliers_refine.size()
              << std::endl;
    // Assign stuff to parent.
    _modelCoeffs = model_coeffs_refine;
    _inliersIndex = inliers_refine;
    _pointIsInPlane.clear();
    // Write inliers indices into map.
    for (const int& index : _inliersIndex) {
        _pointIsInPlane.insert(std::pair<int, bool>(index, true));
    }

    // Set flag so we know that segmentation is done.
    is_segmented_ = true;
}
}
