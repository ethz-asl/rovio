/*
 * DepthCalculationStatistics.h
 *
 *  Created on: Jan 18, 2017
 *      Author: wilczynski
 */

#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "monolidar_fusion/DepthCalcStatsSinglePoint.h"

namespace Mono_Lidar {
class DepthCalculationStatistics {
public:
    DepthCalculationStatistics();

    inline void SetPointCount(int value) {
        _pointCount = value;
        _acc_pointCount += value;
    }
    inline void AddSuccess() {
        _success++;
        _acc_success++;
    }
    inline void AddRadiusSearchInsufficientPoints() {
        _radiussearch_insufficient_points++;
        _acc_radiussearch_insufficient_points++;
    }
    inline void AddHistogramNoLocalMax() {
        _histogram_no_local_max++;
        _acc_histogram_no_local_max++;
    }
    inline void AddTresholdDepthGlobalGreaterMax() {
        _treshold_depth_global_greater_max++;
        _acc_treshold_depth_global_greater_max++;
    }
    inline void AddTresholdDepthGlobalSmallerMin() {
        _treshold_depth_global_smaller_min++;
        _acc_treshold_depth_global_smaller_min++;
    }
    inline void AddTresholdDepthLocalGreaterMax() {
        _treshold_depth_local_greater_max++;
        _acc_treshold_depth_local_greater_max++;
    }
    inline void AddTresholdDepthLocalSmallerMin() {
        _treshold_depth_local_smaller_min++;
        _acc_treshold_depth_local_smaller_min++;
    }
    inline void AddTriangleNotPlanar() {
        _triangle_not_planar++;
        _acc_triangle_not_planar++;
    }
    inline void AddTriangleNotPlanarInsufficientPoints() {
        _triangle_not_planar_insufficient_points++;
        _acc_triangle_not_planar_insufficient_points++;
    }
    inline void AddCornerBehindCamera() {
        _corner_behind_camera++;
        _acc_corner_behind_camera++;
    }
    inline void AddPlaneViewrayNotOrthogonal() {
        _plane_viewray_not_orthogonal++;
        _acc_plane_viewray_not_orthogonal++;
    }
    inline void AddPCAIsPoint() {
        _pca_IsPoint++;
        _acc_pca_IsPoint++;
    }
    inline void AddPCAIsLine() {
        _pca_IsLine++;
        _acc_pca_IsLine++;
    }
    inline void AddPCAIsCubic() {
        _pca_IsCubic++;
        _acc_pca_IsCubic++;
    }
    inline void AddSuccessRoad() {
        _success_road++;
        _acc_success_road++;
    }
    inline void AddInsufficientRoadPoints() {
        _insufficient_road_points++;
        _acc_insufficient_road_points++;
    }
    inline void AddRegionGrowingInsufficientPoints() {
        _region_growing_insufficient_points++;
        _acc_region_growing_insufficient_points++;
    }
    inline void AddRegionGrowingNearestSeedNotAvailable() {
        _region_growing_nearest_seed_not_availaible++;
        _acc_region_growing_nearest_seed_not_availaible++;
    }
    inline void AddRegionGrowingSeedsOutOfRange() {
        _acc_region_growing_seeds_out_of_range++;
        _acc_region_growing_seeds_out_of_range++;
    }
    inline void AddSuccessRegionGrowing() {
        _region_growing_success++;
        _acc_region_growing_success++;
    }
    inline void AddUnspecified() {
        _unspecified++;
        _acc_unspecified++;
    }

    inline int getPointCount() {
        return _pointCount;
    }
    inline int getSuccess() {
        return _success;
    }
    inline int getRadiusSearchInsufficientPoints() {
        return _radiussearch_insufficient_points;
    }
    inline int getHistogramNoLocalMax() {
        return _histogram_no_local_max;
    }
    inline int getTresholdDepthGlobalGreaterMax() {
        return _treshold_depth_global_greater_max;
    }
    inline int getTresholdDepthGlobalSmallerMin() {
        return _treshold_depth_global_smaller_min;
    }
    inline int getTresholdDepthLocalGreaterMax() {
        return _treshold_depth_global_greater_max;
    }
    inline int getTresholdDepthLocalSmallerMin() {
        return _treshold_depth_global_smaller_min;
    }
    inline int getTriangleNotPlanar() {
        return _triangle_not_planar;
    }
    inline int getTriangleNotPlanarInsufficientPoints() {
        return _triangle_not_planar_insufficient_points;
    }
    inline int getCornerBehindCamera() {
        return _corner_behind_camera;
    }
    inline int getPlaneViewrayNotOrthogonal() {
        return _plane_viewray_not_orthogonal;
    }
    inline int getPCAIsPoint() {
        return _pca_IsPoint;
    }
    inline int getPCAIsLine() {
        return _pca_IsLine;
    }
    inline int getPCAIsCubic() {
        return _pca_IsCubic;
    }
    inline int getSuccessRoad() {
        return _success_road;
    }
    inline int getInsufficientRoadPoints() {
        return _insufficient_road_points;
    }
    inline int getRegionGrowingInsufficientPoints() {
        return _region_growing_insufficient_points;
    }
    inline int getRegionGrowingNearestSeedNotAvailable() {
        return _region_growing_nearest_seed_not_availaible;
    }
    inline int getRegionGrowingSeedsOutOfRange() {
        return _acc_region_growing_seeds_out_of_range;
    }
    inline int getSuccessRegionGrowing() {
        return _region_growing_success;
    }
    inline int getUnspecified() {
        return _unspecified;
    }

    inline std::vector<std::shared_ptr<DepthCalcStatsSinglePoint>>& getPointStats() {
        return _points;
    }

    void AddPoint(const std::shared_ptr<DepthCalcStatsSinglePoint>& point);

    void Clear();

    void ToFile(std::ostream& os);

private:
    std::vector<std::shared_ptr<DepthCalcStatsSinglePoint>> _points;
    int _pointCount;
    int _success;
    int _radiussearch_insufficient_points;
    int _histogram_no_local_max;
    int _treshold_depth_global_greater_max;
    int _treshold_depth_global_smaller_min;
    int _treshold_depth_local_greater_max;
    int _treshold_depth_local_smaller_min;
    int _triangle_not_planar;
    int _triangle_not_planar_insufficient_points;
    int _corner_behind_camera;
    int _plane_viewray_not_orthogonal;
    int _pca_IsPoint;
    int _pca_IsLine;
    int _pca_IsCubic;
    int _success_road;
    int _insufficient_road_points;
    int _region_growing_insufficient_points;
    int _region_growing_nearest_seed_not_availaible;
    int _region_growing_seeds_out_of_range;
    int _region_growing_success;
    int _unspecified;

    // accumulated points
    int _acc_frames;
    int _acc_pointCount;
    int _acc_success;
    int _acc_radiussearch_insufficient_points;
    int _acc_histogram_no_local_max;
    int _acc_treshold_depth_global_greater_max;
    int _acc_treshold_depth_global_smaller_min;
    int _acc_treshold_depth_local_greater_max;
    int _acc_treshold_depth_local_smaller_min;
    int _acc_triangle_not_planar;
    int _acc_triangle_not_planar_insufficient_points;
    int _acc_corner_behind_camera;
    int _acc_plane_viewray_not_orthogonal;
    int _acc_pca_IsPoint;
    int _acc_pca_IsLine;
    int _acc_pca_IsCubic;
    int _acc_success_road;
    int _acc_insufficient_road_points;
    int _acc_region_growing_insufficient_points;
    int _acc_region_growing_nearest_seed_not_availaible;
    int _acc_region_growing_seeds_out_of_range;
    int _acc_region_growing_success;
    int _acc_unspecified;


    friend std::ostream& operator<<(std::ostream& os, const DepthCalculationStatistics& d) {
        using namespace std;

        os << "--- DepthCalcStats : " << endl;
        os << "Current Frame: " << endl;
        os << "Points Count: " << d._pointCount << endl;
        os << "Success: " << d._success << endl;
        os << "Region growing success: " << d._region_growing_success << endl;
        os << "Radius search insufficient points: " << d._radiussearch_insufficient_points << endl;
        os << "Histogram no Local max: " << d._histogram_no_local_max << endl;
        os << "Triangle not planar: " << d._triangle_not_planar << endl;
        os << "Triangle not planar insufficient points: " << d._triangle_not_planar_insufficient_points << endl;
        os << "Plane viewray not orthogonal: " << d._plane_viewray_not_orthogonal << endl;
        os << "Treshold depth global greater max: " << d._treshold_depth_global_greater_max << endl;
        os << "Trehsold depth global smaller min: " << d._treshold_depth_global_smaller_min << endl;
        os << "Treshold depth local greater max: " << d._treshold_depth_local_greater_max << endl;
        os << "Trehsold depth local smaller min: " << d._treshold_depth_local_smaller_min << endl;
        os << "Corner behind camera: " << d._corner_behind_camera << endl;
        os << "PCA is point: " << d._pca_IsPoint << endl;
        os << "PCA is line: " << d._pca_IsLine << endl;
        os << "PCA is cubic: " << d._pca_IsCubic << endl;
        os << "Success road: " << d._success_road << endl;
        os << "Insufficient road points: " << d._insufficient_road_points << endl;
        os << "Region growing insufficient points: " << d._region_growing_insufficient_points << endl;
        os << "Region growing Nearest seed not available: " << d._region_growing_nearest_seed_not_availaible << endl;
        os << "Region growing seeds out of range: " << d._region_growing_seeds_out_of_range << endl;
        os << "Unspecified: " << d._unspecified << endl;
        os << endl;

        return os;
    }
};
}
