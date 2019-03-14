/*
 * RansacPlane.h
 *
 *  Created on: Mar 6, 2017
 *      Author: wilczynski
 */
/*
 * RansacPlane.h
 *
 *  Created on: Mar 6, 2017
 *      Author: wilczynski
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include "DepthEstimatorParameters.h"

#include <set>

///@brief forward declaration
namespace cv {
class Mat;
}

namespace Mono_Lidar {

/**
*  @class GroundPlane
*  @par
*
*  Abstract groundplane for depth estimation, may be used with ransac or semantics
*/
class GroundPlane {
public: // Public classes/enums/types etc...
    using Ptr = std::shared_ptr<GroundPlane>;

    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;

    struct ExceptionPclInvalid : public std::exception {
        virtual const char* what() const throw() {
            return "In GroundPlane: Input pointcloud is invalid";
        }
    };

public: // Attributes.
public: // Public methods.
    // Default constructor.
    explicit GroundPlane() = default;

    // Default destructor.
    virtual ~GroundPlane() = default;

    // Default move.
    GroundPlane(GroundPlane&& other) = default;
    GroundPlane& operator=(GroundPlane&& other) = default;

    // Default copy.
    GroundPlane(const GroundPlane& other) = default;
    GroundPlane& operator=(const GroundPlane& other) = default;


    // Interface for calculate inliers
    virtual void CalculateInliersPlane(const Cloud::ConstPtr& pointCloud) = 0;

    /*
     * Function that will test if ransac was applied at least once
     */
    bool isSegmented() const {
        return is_segmented_;
    }

    /*
     * The lastly estimated plane coefficients in the order: a, b, c, d
     */
    inline Eigen::Vector4f& getModelCoeffs() {
        return _modelCoeffs;
    }

    /*
     * Check if the point from the raw pointcloud with the given index has been declared as an inliner
     * @param index [in] The Index of the given point from the original point cloud
     * @return True if the point with the index is an inlier
     */
    bool CheckPointInPlane(const int index) const;

    /*
     * Calculates a matrix (3 x poinSize) which stores all inlies from the plane estimation.
     * @cloudMatrix [in] Full PointCloud in matrix form (3 x pointSize)
     * @inliers [in] List of indices with refer to the inlier points from estimaion
     * @matrix [out] Calculated Matrix (3 x InlierSize) which contains all inlier points
     */
    void getMatrixFromIndices(const Eigen::MatrixXf& cloudMatrix,
                              const std::vector<int>& inliers,
                              Eigen::MatrixXf& matrix) const;

    /*
     * Returns a list which stores the index of the points which have been estimated as plane inliers.
     * The index relaes to the original point cloud.
     */
    inline std::vector<int>& getInlinersIndex() {
        return _inliersIndex;
    }

protected:
    bool is_segmented_; ///< flag that must be set when segmentation was done.

    Eigen::Vector4f _modelCoeffs;

    // Lookup table to check if a point with the given index (index from the full/no resized lidar point cloud) is a
    // point of the plane
    std::map<int, bool> _pointIsInPlane;

    std::vector<int> _inliersIndex;
};
/*
 * Class used for plane estimation out of a given distorted point cloud
 *
 * In this context the class is used for segmenting the ground plane (the road in the context of a vehicle)
 * out of a pointcloud representing a whole scenery.
 */
class RansacPlane : public GroundPlane {
public:
    using Ptr = std::shared_ptr<RansacPlane>;


    RansacPlane() {
        _planeDistanceTreshold = 0;
        _planeMaxIteraions = 0;
        _planeProbability = 0.999;
        _planeRefinementDistance = 10000;
        _doUsePlaneRefinement = false;
        _inliersIndex.clear();
        _pointIsInPlane.clear();
    }
    RansacPlane(const std::shared_ptr<Mono_Lidar::DepthEstimatorParameters>& parameters);


    /*
     * Estimates a plane into a given pointCloud using RANSAC. The results are stored in the class object ad are
     * obtained using the getter methods.
     * @param pointCloud [in] The pointcloud used for the estimation
     * @param cloudMatrix [in] The same pointCloud in matrix notation. It's calculated from outside due to optimization
     */
    void CalculateInliersPlane(const Cloud::ConstPtr& pointCloud) override;

private:
    double _planeDistanceTreshold;
    int _planeMaxIteraions;
    bool _doUsePlaneRefinement;
    double _planeRefinementDistance;
    double _planeProbability;
};


/**
*  @class SemanticPlane
*  @par
*
*  Segment plane with sematnic information, then fit plane to all points
*/
class SemanticPlane : public GroundPlane {
public: // Public classes/enums/types etc...
    struct Camera {
        double f;
        double cu;
        double cv;

        Eigen::Affine3d transform_cam_lidar;

        Eigen::Matrix3d getIntrinsics() const {
            Eigen::Matrix3d out;
            out << f, 0, cu, 0, f, cv, 0, 0, 1;
            return out;
        }
    };

public: // Public methods.
    // Default constructor.
    explicit SemanticPlane(const cv::Mat& img, Camera cam, std::set<int> groundplane_label, double inlier_threshold);

    // Default destructor.
    ~SemanticPlane() = default;

    // Default move.
    SemanticPlane(SemanticPlane&& other) = default;
    SemanticPlane& operator=(SemanticPlane&& other) = default;

    // Default copy.
    SemanticPlane(const SemanticPlane& other) = default;
    SemanticPlane& operator=(const SemanticPlane& other) = default;

    void CalculateInliersPlane(const Cloud::ConstPtr& pointCloud) override;

private:
    std::unique_ptr<cv::Mat> semantic_image_;
    Camera cam_;

    double inlier_threshold_{0.1};

    std::set<int> groundplane_label_{6, 7, 8, 9};
};
}
