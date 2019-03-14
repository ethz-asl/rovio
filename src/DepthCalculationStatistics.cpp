/*
 * DepthCalculationStatistics.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/DepthCalculationStatistics.h"

namespace Mono_Lidar {

DepthCalculationStatistics::DepthCalculationStatistics() {
    Clear();

    _acc_frames = 0;
    _acc_pointCount = 0;
    _acc_success = 0;
    _acc_radiussearch_insufficient_points = 0;
    _acc_histogram_no_local_max = 0;
    _acc_treshold_depth_global_greater_max = 0;
    _acc_treshold_depth_global_smaller_min = 0;
    _acc_treshold_depth_local_greater_max = 0;
    _acc_treshold_depth_local_smaller_min = 0;
    _acc_triangle_not_planar = 0;
    _acc_triangle_not_planar_insufficient_points = 0;
    _acc_corner_behind_camera = 0;
    _acc_plane_viewray_not_orthogonal = 0;
    _acc_pca_IsPoint = 0;
    _acc_pca_IsLine = 0;
    _acc_pca_IsCubic = 0;
    _acc_insufficient_road_points = 0;
}

void DepthCalculationStatistics::AddPoint(const std::shared_ptr<DepthCalcStatsSinglePoint>& point) {
    switch (point->_calcResult) {
    case DepthResultType::Success:
        AddSuccess();
        break;
    case DepthResultType::RadiusSearchInsufficientPoints:
        AddRadiusSearchInsufficientPoints();
        break;
    case DepthResultType::HistogramNoLocalMax:
        AddHistogramNoLocalMax();
        break;
    case DepthResultType::TresholdDepthGlobalGreaterMax:
        AddTresholdDepthGlobalGreaterMax();
        break;
    case DepthResultType::TresholdDepthGlobalSmallerMin:
        AddTresholdDepthGlobalSmallerMin();
        break;
    case DepthResultType::TresholdDepthLocalGreaterMax:
        AddTresholdDepthLocalGreaterMax();
        break;
    case DepthResultType::TresholdDepthLocalSmallerMin:
        AddTresholdDepthLocalSmallerMin();
        break;
    case DepthResultType::TriangleNotPlanar:
        AddTriangleNotPlanar();
        break;
    case DepthResultType::TriangleNotPlanarInsufficientPoints:
        AddTriangleNotPlanarInsufficientPoints();
        break;
    case DepthResultType::CornerBehindCamera:
        AddCornerBehindCamera();
        break;
    case DepthResultType::PlaneViewrayNotOrthogonal:
        AddPlaneViewrayNotOrthogonal();
        break;
    case DepthResultType::PcaIsPoint:
        AddPCAIsPoint();
        break;
    case DepthResultType::PcaIsLine:
        AddPCAIsLine();
        break;
    case DepthResultType::PcaIsCubic:
        AddPCAIsCubic();
        break;
    case DepthResultType::SuccessRoad:
        AddSuccessRoad();
        break;
    case DepthResultType::InsufficientRoadPoints:
        AddInsufficientRoadPoints();
        break;
    case DepthResultType::RegionGrowingInsufficientPoints:
        AddRegionGrowingInsufficientPoints();
        break;
    case DepthResultType::RegionGrowingNearestSeedNotAvailable:
        AddRegionGrowingNearestSeedNotAvailable();
        break;
    case DepthResultType::RegionGrowingSeedsOutOfRange:
        AddRegionGrowingSeedsOutOfRange();
        break;
    case DepthResultType::SuccessRegionGrowing:
        AddSuccessRegionGrowing();
        break;
    case DepthResultType::Unspecified:
        AddUnspecified();
        break;
    }

    _points.push_back(point);
}

void DepthCalculationStatistics::DepthCalculationStatistics::Clear() {
    _acc_frames++;

    this->_pointCount = 0;
    this->_histogram_no_local_max = 0;
    this->_radiussearch_insufficient_points = 0;
    this->_success = 0;
    this->_treshold_depth_global_greater_max = 0;
    this->_treshold_depth_global_smaller_min = 0;
    this->_treshold_depth_local_greater_max = 0;
    this->_treshold_depth_local_smaller_min = 0;
    this->_triangle_not_planar = 0;
    this->_corner_behind_camera = 0;
    this->_triangle_not_planar_insufficient_points = 0;
    this->_plane_viewray_not_orthogonal = 0;
    this->_pca_IsCubic = 0;
    this->_pca_IsLine = 0;
    this->_pca_IsPoint = 0;
    this->_success_road = 0;
    this->_insufficient_road_points = 0;
    this->_points.clear();
    this->_region_growing_insufficient_points = 0;
    this->_region_growing_nearest_seed_not_availaible = 0;
    this->_region_growing_seeds_out_of_range = 0;
    this->_region_growing_success = 0;
    this->_unspecified = 0;
}

void DepthCalculationStatistics::ToFile(std::ostream& os) {
    using namespace std;

    os << "Accumulated frames: " << endl;
    os << "Points Count: " << _acc_pointCount << endl;
    os << "Success: " << _acc_success << endl;
    os << "Radius search insufficient points: " << _acc_radiussearch_insufficient_points << endl;
    os << "Histogram no Local max: " << _acc_histogram_no_local_max << endl;
    os << "Triangle not planar: " << _acc_triangle_not_planar << endl;
    os << "Triangle not planar insufficient points: " << _acc_triangle_not_planar_insufficient_points << endl;
    os << "Plane viewray not orthogonal: " << _acc_plane_viewray_not_orthogonal << endl;
    os << "Treshold depth global greater max: " << _acc_treshold_depth_global_greater_max << endl;
    os << "Trehsold depth global smaller min: " << _acc_treshold_depth_global_smaller_min << endl;
    os << "Treshold depth local greater max: " << _acc_treshold_depth_local_greater_max << endl;
    os << "Trehsold depth local smaller min: " << _acc_treshold_depth_local_smaller_min << endl;
    os << "Corner behind camera: " << _acc_corner_behind_camera << endl;
    os << "PCA is point: " << _acc_pca_IsPoint << endl;
    os << "PCA is line: " << _acc_pca_IsLine << endl;
    os << "PCA is cubic: " << _acc_pca_IsCubic << endl;
    os << "Success road: " << _acc_success_road << endl;
    os << "Insufficient road points: " << _acc_insufficient_road_points << endl;
    os << "Region growing insufficient points: " << _acc_region_growing_insufficient_points << endl;
    os << "Region growing nearest seeds not available: " << _acc_region_growing_nearest_seed_not_availaible << endl;
    os << "Region growing seeds out of range: " << _acc_region_growing_seeds_out_of_range << endl;
    os << "Region growing success: " << _region_growing_success << endl;
    os << "Unspecified: " << _acc_unspecified << endl;
    os << endl;

    // Write average statistics relative to complete point count
    os << "Average by all points: " << endl;
    double av_pointCount = (double)_acc_pointCount / _acc_frames;
    double av_success = (double)_acc_success / _acc_pointCount * 100;
    double av_radiussearch_insufficient_points = (double)_acc_radiussearch_insufficient_points / _acc_pointCount * 100;
    double av_histogram_no_local_max = (double)_acc_histogram_no_local_max / _acc_pointCount * 100;
    double av_triangle_not_planar = (double)_acc_triangle_not_planar / _acc_pointCount * 100;
    double av_triangle_not_planar_insufficient_points =
        (double)_acc_triangle_not_planar_insufficient_points / _acc_pointCount * 100;
    double av_plane_viewray_not_orthogonal = (double)_acc_plane_viewray_not_orthogonal / _acc_pointCount * 100;
    double av_treshold_depth_global_greater_max =
        (double)_acc_treshold_depth_global_greater_max / _acc_pointCount * 100;
    double av_treshold_depth_global_smaller_min =
        (double)_acc_treshold_depth_global_smaller_min / _acc_pointCount * 100;
    double av_treshold_depth_local_greater_max = (double)_acc_treshold_depth_local_greater_max / _acc_pointCount * 100;
    double av_treshold_depth_local_smaller_min = (double)_acc_treshold_depth_local_smaller_min / _acc_pointCount * 100;
    double av_corner_behind_camera = (double)_acc_corner_behind_camera / _acc_pointCount * 100;
    double av_pca_IsPoint = (double)_acc_pca_IsPoint / _acc_pointCount * 100;
    double av_pca_IsLine = (double)_acc_pca_IsLine / _acc_pointCount * 100;
    double av_pca_IsCubic = (double)_acc_pca_IsCubic / _acc_pointCount * 100;
    double av_success_road = (double)_acc_success_road / _acc_pointCount * 100;
    double av_insufficient_road_points = (double)_acc_insufficient_road_points / _acc_pointCount * 100;
    double av_region_growing_insufficient_points =
        (double)_acc_region_growing_insufficient_points / _acc_pointCount * 100;
    double av_region_growing_nearest_seed_not_availaible =
        (double)_acc_region_growing_nearest_seed_not_availaible / _acc_pointCount * 100;
    double av_region_growing_seeds_out_of_range =
        (double)_acc_region_growing_seeds_out_of_range / _acc_pointCount * 100;
    double av_region_growing_success = (double)_acc_region_growing_success / _acc_pointCount * 100;
    double av_unspecified = (double)_acc_unspecified / _acc_pointCount * 100;

    os << "Points Count: " << av_pointCount << endl;
    os << "Success: " << av_success << endl;
    os << "Radius search insufficient points: " << av_radiussearch_insufficient_points << endl;
    os << "Histogram no Local max: " << av_histogram_no_local_max << endl;
    os << "Triangle not planar: " << av_triangle_not_planar << endl;
    os << "Triangle not planar insufficient points: " << av_triangle_not_planar_insufficient_points << endl;
    os << "Plane viewray not orthogonal: " << av_plane_viewray_not_orthogonal << endl;
    os << "Treshold depth global greater max: " << av_treshold_depth_global_greater_max << endl;
    os << "Trehsold depth global smaller min: " << av_treshold_depth_global_smaller_min << endl;
    os << "Treshold depth local greater max: " << av_treshold_depth_local_greater_max << endl;
    os << "Trehsold depth local smaller min: " << av_treshold_depth_local_smaller_min << endl;
    os << "Corner behind camera: " << av_corner_behind_camera << endl;
    os << "PCA is point: " << av_pca_IsPoint << endl;
    os << "PCA is line: " << av_pca_IsLine << endl;
    os << "PCA is cubic: " << av_pca_IsCubic << endl;
    os << "Success Road: " << av_success_road << endl;
    os << "Insufficient road points: " << av_insufficient_road_points << endl;
    os << "Region growing insufficient points: " << av_region_growing_insufficient_points << endl;
    os << "Region growing nearest seed not available: " << av_region_growing_nearest_seed_not_availaible << endl;
    os << "Region growing seeds out of range: " << av_region_growing_seeds_out_of_range << endl;
    os << "Region growing Success: " << av_region_growing_success << endl;
    os << "Unspecified: " << av_unspecified << endl;
    os << endl;

    // Write average statistics relative to point count wich are captured by the lidar
    os << "Average by captured points by lidar: " << endl;
    double acc_sufficien_point_count = _acc_pointCount - _acc_radiussearch_insufficient_points;
    double av2_sufficientPointCount = (double)acc_sufficien_point_count / _acc_frames;
    double av2_success = (double)_acc_success / acc_sufficien_point_count * 100;
    double av2_radiussearch_insufficient_points =
        (double)_acc_radiussearch_insufficient_points / acc_sufficien_point_count * 100;
    double av2_histogram_no_local_max = (double)_acc_histogram_no_local_max / acc_sufficien_point_count * 100;
    double av2_triangle_not_planar = (double)_acc_triangle_not_planar / acc_sufficien_point_count * 100;
    double av2_triangle_not_planar_insufficient_points =
        (double)_acc_triangle_not_planar_insufficient_points / acc_sufficien_point_count * 100;
    double av2_plane_viewray_not_orthogonal =
        (double)_acc_plane_viewray_not_orthogonal / acc_sufficien_point_count * 100;
    double av2_treshold_depth_global_greater_max =
        (double)_acc_treshold_depth_global_greater_max / acc_sufficien_point_count * 100;
    double av2_treshold_depth_global_smaller_min =
        (double)_acc_treshold_depth_global_smaller_min / acc_sufficien_point_count * 100;
    double av2_treshold_depth_local_greater_max =
        (double)_acc_treshold_depth_local_greater_max / acc_sufficien_point_count * 100;
    double av2_treshold_depth_local_smaller_min =
        (double)_acc_treshold_depth_local_smaller_min / acc_sufficien_point_count * 100;
    double av2_corner_behind_camera = (double)_acc_corner_behind_camera / acc_sufficien_point_count * 100;
    double av2_pca_IsPoint = (double)_acc_pca_IsPoint / acc_sufficien_point_count * 100;
    double av2_pca_IsLine = (double)_acc_pca_IsLine / acc_sufficien_point_count * 100;
    double av2_pca_IsCubic = (double)_acc_pca_IsCubic / acc_sufficien_point_count * 100;
    double av2_success_road = (double)_acc_success_road / acc_sufficien_point_count * 100;
    double av2_insufficient_road_points = (double)_acc_insufficient_road_points / acc_sufficien_point_count * 100;
    double av2_region_growing_insufficient_points =
        (double)_acc_region_growing_insufficient_points / acc_sufficien_point_count * 100;
    double av2_region_growing_nearest_seed_not_availaible =
        (double)_acc_region_growing_nearest_seed_not_availaible / acc_sufficien_point_count * 100;
    double av2_region_growing_seeds_out_of_range =
        (double)_acc_region_growing_seeds_out_of_range / acc_sufficien_point_count * 100;
    double av2_region_growing_success = (double)_acc_region_growing_success / acc_sufficien_point_count * 100;
    double av2_unspecified = (double)_acc_unspecified / acc_sufficien_point_count * 100;

    os << "Points captured by lidar Count: " << av2_sufficientPointCount << endl;
    os << "Success: " << av2_success << endl;
    os << "Radius search insufficient points: " << av2_radiussearch_insufficient_points << endl;
    os << "Histogram no Local max: " << av2_histogram_no_local_max << endl;
    os << "Triangle not planar: " << av2_triangle_not_planar << endl;
    os << "Triangle not planar insufficient points: " << av2_triangle_not_planar_insufficient_points << endl;
    os << "Plane viewray not orthogonal: " << av2_plane_viewray_not_orthogonal << endl;
    os << "Treshold depth global greater max: " << av2_treshold_depth_global_greater_max << endl;
    os << "Trehsold depth global smaller min: " << av2_treshold_depth_global_smaller_min << endl;
    os << "Treshold depth local greater max: " << av2_treshold_depth_local_greater_max << endl;
    os << "Trehsold depth local smaller min: " << av2_treshold_depth_local_smaller_min << endl;
    os << "Corner behind camera: " << av2_corner_behind_camera << endl;
    os << "PCA is point: " << av2_pca_IsPoint << endl;
    os << "PCA is line: " << av2_pca_IsLine << endl;
    os << "PCA is cubic: " << av2_pca_IsCubic << endl;
    os << "Success Road: " << av2_success_road << endl;
    os << "Insufficient road points: " << av2_insufficient_road_points << endl;
    os << "Region growing insufficient points: " << av2_region_growing_insufficient_points << endl;
    os << "Region growing nearest seed not available: " << av2_region_growing_nearest_seed_not_availaible << endl;
    os << "Region growing seeds out of range: " << av2_region_growing_seeds_out_of_range << endl;
    os << "Region growing Success: " << av2_region_growing_success << endl;
    os << "Unspecified: " << av2_unspecified << endl;
    os << endl;
}
}
