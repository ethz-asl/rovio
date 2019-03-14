/*
 * DepthEstimator.h
 *
 *  Created on: Dec 6, 2016
 *      Author: wilczynski
 */

#pragma once

#include <memory>
#include <Eigen/Eigen> // IWYU pragma: keep
#include <Eigen/StdVector>
//#include <opencv/cxcore.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "monolidar_fusion/DepthCalcStatsSinglePoint.h"
#include "monolidar_fusion/DepthCalculationStatistics.h"
#include "monolidar_fusion/DepthEstimatorParameters.h"
#include "monolidar_fusion/HelperLidarRowSegmentation.h"
#include "monolidar_fusion/LinePlaneIntersectionBase.h"
#include "monolidar_fusion/NeighborFinderBase.h"
#include "monolidar_fusion/PlaneEstimationCalcMaxSpanningTriangle.h"
#include "monolidar_fusion/PlaneEstimationCheckPlanar.h"
#include "monolidar_fusion/PointcloudData.h"
#include "monolidar_fusion/RansacPlane.h"
#include "monolidar_fusion/RoadDepthEstimatorBase.h"
#include "monolidar_fusion/TresholdDepthGlobal.h"
#include "monolidar_fusion/TresholdDepthLocal.h"
#include "monolidar_fusion/camera_pinhole.h"
#include "monolidar_fusion/eDepthResultType.h"

namespace Mono_Lidar {

class DepthEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    std::map<DepthResultType, std::string> DepthResultTypeMap{
        {Success, "Success"},
        {RadiusSearchInsufficientPoints, "RadiusSearchInsufficientPoints"},
        {HistogramNoLocalMax, "HistogramNoLocalMax"},
        {TresholdDepthGlobalGreaterMax, "TresholdDepthGlobalGreaterMax"},
        {TresholdDepthGlobalSmallerMin, "TresholdDeptGlobalhSmallerMin"},
        {TresholdDepthLocalGreaterMax, "TresholdDepthLocalGreaterMax"},
        {TresholdDepthLocalSmallerMin, "TresholdDeptLocalhSmallerMin"},
        {TriangleNotPlanar, "TriangleNotPlanar"},
        {TriangleNotPlanarInsufficientPoints, "TriangleNotPlanarInsufficientPoints"},
        {CornerBehindCamera, "CornerBehindCamera"},
        {PlaneViewrayNotOrthogonal, "PlaneViewrayNotOrthogonal"},
        {PcaIsPoint, "PcaIsPoint"},
        {PcaIsLine, "PcaIsLine"},
        {PcaIsCubic, "PcaIsCubic"},
        {InsufficientRoadPoints, "InsufficientRoadPoints"}};

    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;
    using UniquePtr = std::unique_ptr<DepthEstimator>;
    using SharedPtr = std::shared_ptr<DepthEstimator>;

    /*
     * Must be called once before using.
     *
     * @param camera Intrinsic camera parameters
     * @param transform_lidar_to_cam Pose of the camera in lidar coordinates
     */
    bool Initialize(const std::shared_ptr<CameraPinhole>& camera, const Eigen::Affine3d& transform_lidar_to_cam);


    bool Initialize(const Eigen::Vector3d& BrBC, const Eigen::Quaterniond& qBC, const Eigen::Matrix3d& K);

    /*
     * Initializes the parameters from a config file. Must be called before class usuage.
     */
    bool InitConfig(const std::string& filePath, const bool printparams = true);

    /*
     * Initializes the parameters with default parameters. Must be called before class usuage.
     */
    bool InitConfig(const bool printparams = false);

    void ActivateDebugMode() {
        _debugMode = true;
    }

    /*
     * Sets the lidar pointcloud and transforms the included points into camera coordinates
     * @param pointCloud pointcloud from velodyne in lidar cs
     */
    void setInputCloud(const Cloud::ConstPtr& pointCloud, GroundPlane::Ptr& ransacPlane);

    /*
     * Gets the parameter object
     */
    std::shared_ptr<DepthEstimatorParameters> getParameters() {
        return _parameters;
    }

    /*
     * Gets the camera object
     */
    std::shared_ptr<CameraPinhole> getCamera() {
        return _camera;
    }

    /*
     * Gets the pose of the camera in lidar coordinates as an affine transformation
     */
    Eigen::Affine3d getTransformLidarToCam() {
        return _transform_lidar_to_cam;
    }

    double getPointDepthCamVisible(int index) {
        return _points._points_cs_camera(2, _points._pointIndex[index]);
    }

    std::shared_ptr<HelperLidarRowSegmentation> getLidarRowSegmentation() {
        return _lidarRowSegmenter;
    }

    /*
     * Returns the original pointcloud in camera coordinates
     */
    void getCloudCameraCs(Cloud::Ptr& pointCloud_cam_cs);

    /*
     * Returns the estimated 3d points (calculated from a given set of points in image cs)
     */
    void getCloudInterpolated(Cloud::Ptr& pointCloud_interpolated);

    /*
     * Returns the estimated 3d points which were interpolated from the estimated ransac-plane/road
     */
    void getCloudInterpolatedPlane(Cloud::Ptr& pointCloud_interpolated_plane);

    /*
     * Returns a point cloud containing all points which were found by the neirest neighbors search
     */
    void getCloudNeighbors(Cloud::Ptr& pointCloud_neighbors);

    /*
     * Returns a point cloud which contains all corner points of the triangles used for point interpolation/depth
     * estimation
     */
    void getCloudTriangleCorners(Cloud::Ptr& pointCloud_triangle_corner);

    /*
     * Gets the lidar point cloud points which are visible in the camera image frame in image cs.
     */
    void getPointsCloudImageCs(Eigen::Matrix2Xd& visiblePointsImageCs);

    /*
     * Gets all points that are estimated as a road by matching it with a plane using ransac
     */
    void getCloudRansacPlane(Cloud::Ptr& pointCloud_plane_ransac);


    /*
     * gets the actual statistics of the last calculated frame
     */
    const DepthCalculationStatistics& getDepthCalcStats();

    /*
    * Calculates the depth of a set of given points
    * The pointcloud and the image must be synchronized in time
    * @param pointCloud [in] 3D pointcloud. Only points inside the camera view cone will be considered
    * @param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row
    * order
    * @param points_depths [out] Depths of the given points in meters
    */
    void CalculateDepth(const Cloud::ConstPtr& pointCloud,
                        const Eigen::Matrix2Xd& points_image_cs,
                        Eigen::VectorXd& points_depths,
                        GroundPlane::Ptr& ransacPlane);

  /*
  * Calculates the depth of a set of given points
  * The pointcloud and the image must be synchronized in time
  * @param pointCloud [in] 3D pointcloud. Only points inside the camera view cone will be considered
  * @param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row
  * order
  * @param points_depths [out] Depths of the given points in meters
  */
  void CalculateDepth(const Cloud::ConstPtr& pointCloud,
                      const Eigen::Matrix2Xd& points_image_cs,
                      Eigen::VectorXd& points_depths);
    /*
    * Calculates the depth of a set of given points
    * The pointcloud and the image must be synchronized in time
    * @param pointCloud [in] 3D pointcloud. Only points inside the camera view cone will be considered
    * @param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row
    * order
    * @param points_depths [out] Depths of the given points in meters
    */
    void CalculateDepth(const Cloud::ConstPtr& pointCloud,
                        const Eigen::Matrix2Xd& points_image_cs,
                        Eigen::VectorXd& points_depths,
                        Eigen::VectorXi& resultType,
                        GroundPlane::Ptr& ransacPlane);

    /*
     * Calculates the depth of a set of given points
     * @param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row
     * order
     * @param points_depths [out] Depths of the given points in meters
     */
    void CalculateDepth(const Eigen::Matrix2Xd& points_image_cs,
                        Eigen::VectorXd& points_depths,
                        const GroundPlane::Ptr& ransacPlane);

    /*
     * Calculates the depth of a set of given points
     * @param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row
     * order
     * @param points_depths [out] Depths of the given points in meters
     */
    void CalculateDepth(const Eigen::Matrix2Xd& points_image_cs,
                        Eigen::VectorXd& points_depths,
                        Eigen::VectorXi& resultType,
                        const GroundPlane::Ptr& ransacPlane);

    /*
     * Calculates the depth of a single image point by intersection of a camera raycast with a hyperplane (created by
     * nearest neighbor lidar points)
     * @param point_image_cs 2d [in] Points in image coordinates
     * @param calculated depth of the image point
     */
    std::pair<DepthResultType, double> CalculateDepth(const Eigen::Vector2d& point_image_cs,
                                                      const GroundPlane::Ptr& ransacPlane,
                                                      std::shared_ptr<DepthCalcStatsSinglePoint> calcStats = NULL);

    void LogDepthCalcStats(const DepthResultType depthResult);
private:
    bool InitializeParameters();

    /*
     * Creates a pointcloud with a list of 3D points
     *
     * @param content List of 3d points
     * @param cloud Created cloud
     */
    void FillCloud(const std::vector<Eigen::Vector3d> content, const Cloud::Ptr& cloud);

    /*
     * Extracts the part of a pointcloud which is visible in a camera frame
     *
     * @param cloud_in Full input cloud
     * @param Cut Visible cloud in a camera which results to a cone
     */
    void CutPointCloud(const Cloud::ConstPtr& cloud_in, const Cloud::Ptr& cloud_out);

    /*
     * Gets the neighbors inside a rect in the image plane
     *
     * @param featurePoint_image_cs 2D feature point in the image plane
     * @param neighborIndices Found indices of the 2D-Neighbors. The indices are based on the visible points in the
     * image plane.
     * @param neighbors 3D-Lidar Positions of the Neighbor points
     * @param Object for statistics
     */
    bool CalculateNeighbors(const Eigen::Vector2d& featurePoint_image_cs,
                            std::vector<int>& neighborIndices,
                            std::vector<Eigen::Vector3d>& neighbors,
                            std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
                            const float scaleX = 1.0,
                            const float scaleY = 1.0);

    bool CalculateDepthSegmentation(const std::vector<Eigen::Vector3d>& neighbors,
                                    const std::vector<int>& neighborsIndex,
                                    std::vector<Eigen::Vector3d>& pointsSegmented,
                                    std::vector<int>& pointsSegmentedIndex,
                                    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats);

    /*
     * Gets the nearest lidar point to the camera.
     * It's described by the point with the smallest z value
     *
     * @param neighbors 3D-Lidar Positions of the Neighbor points
     * @param neighborsIndex Found indices of the Neighbors. The indices are based on the visible points in the image
     * plane.
     * @param nearestPoint The 3D Lidar coordinates of the nearest point from the given neighbors
     * @param nearestPointIndex The index (visible points) of the found nearest point
     *
     */
    bool CalculateNearestPoint(const std::vector<Eigen::Vector3d>& neighbors,
                               const std::vector<int>& neighborsIndex,
                               Eigen::Vector3d& nearestPoint,
                               int& nearestPointIndex);

    int CalcDepthSegmentionRegionGrowing(const Eigen::Vector2d& featurePoint_image_cs,
                                         std::vector<Eigen::Vector3d>& neighborsSegmented,
                                         std::vector<int>& imgNeighborsSegmentedIndex);

    bool CalculateDepthSegmentationPlane(const std::vector<Eigen::Vector3d>& neighbors,
                                         const std::vector<int> neighborIndices,
                                         std::vector<Eigen::Vector3d>& pointsSegmented,
                                         std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
                                         GroundPlane::Ptr ransacPlane);

    std::pair<DepthResultType, double> CalculateDepthSegmented(const Eigen::Vector2d& point_image_cs,
                                                               const std::vector<Eigen::Vector3d>& neighborsSegmented,
                                                               std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
                                                               const bool checkPlanarTriangle = false);

    /**
     * Transforms the pointcloud (lidar cs) into the camera cs and projects it's points into the image frame
     * @param lidar_to_cam Affine transformation from lidar to camera cooridnates
     */
    void Transform_Cloud_LidarToCamera(const Cloud::ConstPtr& cloud_lidar_cs, const Eigen::Affine3d& lidar_to_cam);

    /**
     * Writes a lidar point cloud into a different format.
     * @param [in] Pointcloud in format matrix2Xd: Each column contains a single lidar point with it's x and y
     * coordinates in the first two rows
     * @param [out] Pointcloud in format matrixxX: Returns a matrix with the image size (img.witdh * img.height) and
     * index of the visible lidar point cloud at the corresponding x-y-coordinates
     */
    // void Transform(const std::vector<Eigen::Vector3d>& in, Eigen::Matrix3d& out);

    // Initialization flags
    /*
     * Flag which determines if the Initialize method has been called
     */
    bool _isInitialized = false;
    bool _isInitializedConfig = false;
    bool _isInitializedPointCloud = false;
    bool _isInitializedKddTree = false;

    // pointcloud variables

    PointcloudData _points; // stored all points of the pointcloud in

    // Following lists of points are used for visualization and debugging purpose
    std::vector<Eigen::Vector3d>
        _points_interpolated; // list of intersection points from camera ray with triangle image plane
    std::vector<Eigen::Vector3d> _points_interpolated_plane; // list of intersection points from camera ray with
                                                             // triangle image plane from he ground plane
    std::vector<Eigen::Vector3d>
        _points_triangle_corners; // list of triangle corners (lidar points) spanning a local patch for a feature point
    std::vector<Eigen::Vector3d> _points_neighbors;   // list of points of all found neighbor points
    std::vector<Eigen::Vector3d> _points_groundplane; // list of points which are estimated as ground plane

    // Following objects are the modules used for depth estimation
    std::shared_ptr<NeighborFinderBase> _neighborFinder;
    //    std::shared_ptr<RansacPlane> _ransacPlane;
    std::shared_ptr<TresholdDepthGlobal> _tresholdDepthGlobal;
    std::shared_ptr<TresholdDepthLocal> _tresholdDepthLocal;
    std::shared_ptr<RoadDepthEstimatorBase> _roadDepthEstimator;
    std::shared_ptr<LinePlaneIntersectionBase> _linePlaneIntersection;
    std::shared_ptr<PlaneEstimationCheckPlanar> _checkPlanarTriangle;
    std::shared_ptr<PlaneEstimationCalcMaxSpanningTriangle> _planeCalcMaxSpanning;
    std::shared_ptr<HelperLidarRowSegmentation> _lidarRowSegmenter;

    // Misc
    bool _debugMode = false;
    std::shared_ptr<DepthEstimatorParameters> _parameters;
    std::shared_ptr<CameraPinhole> _camera;
    Eigen::Affine3d _transform_lidar_to_cam; // transformation to switch points from lidar frame into cam frame
    Eigen::Affine3d _transform_cam_to_lidar; // inverse of _transform_lidar_to_cam
    std::ofstream _debugFile;
    DepthCalculationStatistics _depthCalcStats;

    bool _isLatestTransformSet = false;
    // image variabes (input image)
    const int _imgWitdh = 720;
    const int _imgHeight = 480;
};


} /* namespace lidorb_ros_tool */
