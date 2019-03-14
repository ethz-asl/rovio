/*
n li * DepthEstirmatorParameters.cpp
 *
 *  Created on: Jan 3, 2017
 *      Author: wilczynski
 */

#include <iostream>
#include <opencv/cxcore.h>
#include <opencv2/core/core.hpp>

#include "monolidar_fusion/DepthEstimatorParameters.h"

namespace Mono_Lidar {

void DepthEstimatorParameters::fromFile(const std::string& filePath) {
    using namespace std;

    cout << "Load settings from: " << filePath << endl << endl;

    cv::FileStorage fSettings(filePath, cv::FileStorage::READ);

    if (!fSettings.isOpened())
        throw("Cant find settings file: " + filePath);

    // Nearest Neighbors search
    do_use_nearestNeighborSearch = (int)fSettings[str_do_use_nearestNeighborSearch];
    nnSearch_count = (int)fSettings[str_nnSearch_count];

    // Pixel area search
    neighbor_search_mode = (int)fSettings[str_neighbor_search_mode];
    pixelarea_search_witdh = (int)fSettings[str_pixelarea_search_witdh];
    pixelarea_search_height = (int)fSettings[str_pixelarea_search_height];
    pixelarea_search_offset_x = (int)fSettings[str_pixelarea_search_offset_x];
    pixelarea_search_offset_y = (int)fSettings[str_pixelarea_search_offset_y];

    // Radius search
    do_use_radiusSearch = (int)fSettings[str_do_use_radiusSearch];
    radiusSearch_radius = (double)fSettings[str_radiusSearch_radius];
    radiusSearch_count_min = (int)fSettings[str_radiusSearch_count_min];

    // Histogram Segmentation
    do_use_histogram_segmentation = (int)fSettings[str_do_use_histogram_segmentation];
    histogram_segmentation_bin_witdh = (double)fSettings[str_histogram_segmentation_bin_witdh];
    histogram_segmentation_min_pointcount = (int)fSettings[str_histogram_segmentation_min_pointcount];

    // Depth segmentation
    do_use_depth_segmentation = (int)fSettings[str_do_use_depth_segmentation];
    depth_segmentation_max_neighbor_distance = (double)fSettings[str_depth_segmentation_max_neighbor_distance];
    depth_segmentation_max_treshold_gradient = (double)fSettings[str_depth_segmentation_max_treshold_gradient];
    depth_segmentation_max_neighbor_distance_gradient =
        (double)fSettings[str_depth_segmentation_max_neighbor_distance_gradient];
    depth_segmentation_max_neighbor_to_seedpoint_distance =
        (double)fSettings[str_depth_segmentation_max_neighbor_to_seedpoint_distance];
    depth_segmentation_max_seedpoint_to_seedpoint_distance_gradient =
        (double)fSettings[str_depth_segmentation_max_seedpoint_to_seedpoint_distance_gradient];
    depth_segmentation_max_seedpoint_to_seedpoint_distance =
        (double)fSettings[str_depth_segmentation_max_seedpoint_to_seedpoint_distance];
    depth_segmentation_max_neighbor_to_seedpoint_distance_gradient =
        (double)fSettings[str_depth_segmentation_max_neighbor_to_seedpoint_distance_gradient];
    depth_segmentation_max_pointcount = (double)fSettings[str_depth_segmentation_max_pointcount];

    // Calculated Depth Treshold
    treshold_depth_enabled = (int)fSettings[str_treshold_depth_enabled];
    treshold_depth_mode = (int)fSettings[str_treshold_depth_mode];
    treshold_depth_max = (int)fSettings[str_treshold_depth_max];
    treshold_depth_min = (int)fSettings[str_treshold_depth_min];

    // Calculated depth local treshold
    treshold_depth_local_enabled = (int)fSettings[str_treshold_depth_local_enabled];
    treshold_depth_local_mode = (int)fSettings[str_treshold_depth_local_mode];
    treshold_depth_local_valuetype = (int)fSettings[str_treshold_depth_local_valuetype];
    treshold_depth_local_value = (double)fSettings[str_treshold_depth_local_value];

    // PCA
    do_use_PCA = (double)fSettings[str_do_use_PCA];
    pca_debug = (int)fSettings[str_pca_debug];
    pca_treshold_3_abs_min = (double)fSettings[str_pca_treshold_3_abs_min];
    pca_treshold_3_2_rel_max = (double)fSettings[str_pca_treshold_3_2_rel_max];
    pca_treshold_2_1_rel_min = (double)fSettings[str_pca_treshold_2_1_rel_min];

    // Ransac Plane Estimation
    do_use_ransac_plane = (int)fSettings[str_do_use_ransac_plane];
    ransac_plane_distance_treshold = (double)fSettings[str_ransac_plane_distance_treshold];
    ransac_plane_max_iterations = (int)fSettings[str_ransac_plane_max_iterations];
    ransac_plane_use_refinement = (int)fSettings[str_ransac_plane_use_refinement];
    ransac_plane_refinement_treshold = (double)fSettings[str_ransac_plane_refinement_treshold];
    ransac_plane_use_camx_treshold = (int)fSettings[str_ransac_plane_use_camx_treshold];
    ransac_plane_treshold_camx = (double)fSettings[str_ransac_plane_treshold_camx];
    ransac_plane_point_distance_treshold = (double)fSettings[str_ransac_plane_point_distance_treshold];
    ransac_plane_debug_visualize = (int)fSettings[str_ransac_plane_debug_visualize];
    ransac_plane_probability = (double)fSettings[str_ransac_plane_probability];

    // Plane depth estimation
    plane_estimator_use_triangle_maximation = (int)fSettings[str_plane_estimator_use_triangle_maximation];
    plane_estimator_z_x_min_relation = (double)fSettings[str_plane_estimator_z_x_min_relation];
    plane_estimator_use_leastsquares = (int)fSettings[str_plane_estimator_use_leastsquares];
    plane_estimator_use_mestimator = (int)fSettings[str_plane_estimator_use_mestimator];

    // Misc
    do_use_cut_behind_camera = (int)fSettings[str_do_use_cut_behind_camera];
    do_use_triangle_size_maximation = (int)fSettings[str_do_use_triangle_size_maximation];
    do_check_triangleplanar_condition = (int)fSettings[str_do_check_triangleplanar_condition];
    triangleplanar_crossnorm_treshold = (double)fSettings[str_triangleplanar_crossnorm_treshold];
    viewray_plane_orthoganality_treshold = (double)fSettings[str_viewray_plane_orthoganality_treshold];
    set_all_depths_to_zero = (int)fSettings[str_set_all_depths_to_zero];

    // Debug
    do_debug_singleFeatures = (int)fSettings[str_do_debug_singleFeatures];
    do_publish_points = (int)fSettings[str_do_publish_points];
    do_depth_calc_statistics = (int)fSettings[str_do_depth_calc_statistics];
}


void DepthEstimatorParameters::print() {
    using namespace std;

    cout << "DepthEstimator parameters: " << endl << endl;

    // Pixel area search
    cout << "_____ pixel area _____ " << endl;
    cout << "neighbor_search_mode: " << neighbor_search_mode << endl;
    cout << "pixelarea_search_witdh: " << pixelarea_search_witdh << endl;
    cout << "pixelarea_search_height: " << pixelarea_search_height << endl;
    cout << "pixelarea_search_offset_x: " << pixelarea_search_offset_x << endl;
    cout << "pixelarea_search_offset_y: " << pixelarea_search_offset_y << endl;
    cout << endl;

    // Nearest Neighbors search
    cout << "_____ Nearest Neighbors search _____" << endl;
    cout << "do_use_nearestNeighborSearch: " << do_use_nearestNeighborSearch << endl;
    cout << "nnSearch_count" << nnSearch_count << endl;
    cout << endl;

    // Radius search
    cout << "_____ Radius search _____" << endl << endl;
    cout << "do_use_radiusSearch: " << do_use_radiusSearch << endl;
    cout << "radiusSearch_radius: " << radiusSearch_radius << endl;
    cout << "radiusSearch_count_min: " << radiusSearch_count_min << endl;
    cout << endl;

    // Histogram Segmentation
    cout << "_____ Histogram Segmentation _____" << endl << endl;
    cout << "do_use_histogram_segmentation: " << do_use_histogram_segmentation << endl;
    cout << "histogram_segmentation_bin_witdh: " << histogram_segmentation_bin_witdh << endl;
    cout << "histogram_segmentation_min_pointcount: " << histogram_segmentation_min_pointcount << endl;
    cout << endl;

    // Depth segmentation
    cout << "_____ Depth segmentation _____" << endl << endl;
    cout << "do_use_depth_segmentation: " << do_use_depth_segmentation << endl;
    cout << "depth_segmentation_max_treshold_gradient: " << depth_segmentation_max_treshold_gradient << endl;
    cout << "depth_segmentation_max_neighbor_distance: " << depth_segmentation_max_neighbor_distance << endl;
    cout << "depth_segmentation_max_neighbor_distance_gradient: " << depth_segmentation_max_neighbor_distance_gradient
         << endl;
    cout << "depth_segmentation_max_neighbor_to_seepoint_distance: "
         << depth_segmentation_max_neighbor_to_seedpoint_distance << endl;
    cout << "depth_segmentation_max_neighbor_to_seepoint_distance_gradient: "
         << depth_segmentation_max_neighbor_to_seedpoint_distance_gradient << endl;
    cout << "depth_segmentation_max_seedpoint_to_seepoint_distance: "
         << depth_segmentation_max_seedpoint_to_seedpoint_distance_gradient << endl;
    cout << "depth_segmentation_max_seedpoint_to_seepoint_distance_gradient: "
         << depth_segmentation_max_seedpoint_to_seedpoint_distance << endl;
    cout << "depth_segmentation_max_pointcount: " << depth_segmentation_max_pointcount << endl;
    cout << endl;

    // Calculated depth treshold
    cout << "____ depth tresholds _____" << endl;
    cout << "treshold_depth_enabled: " << treshold_depth_enabled << endl;
    cout << "treshold_depth_mode: " << treshold_depth_mode << endl;
    cout << "treshold_depth_max: " << treshold_depth_max << endl;
    cout << "treshold_depth_min: " << treshold_depth_min << endl;
    cout << endl;

    // Calculated depth treshold local
    cout << "_____ depth local reshold _____" << endl;
    cout << "treshold_depth_local_enabled: " << treshold_depth_local_enabled << endl;
    cout << "treshold_depth_local_mode: " << treshold_depth_local_mode << endl;
    cout << "treshold_depth_local_valuetype: " << treshold_depth_local_valuetype << endl;
    cout << "treshold_depth_local_value: " << treshold_depth_local_value << endl;
    cout << endl;

    // PCA
    cout << "_____ PCA _____ " << endl;
    cout << "do_use_PCA: " << do_use_PCA << endl;
    cout << "pca_debug: " << pca_debug << endl;
    cout << "pca_treshold_3_abs_min: " << pca_treshold_3_abs_min << endl;
    cout << "pca_treshold_3_2_rel_max: " << pca_treshold_3_2_rel_max << endl;
    cout << "pca_treshold_2_1_rel_min: " << pca_treshold_2_1_rel_min << endl;
    cout << endl;

    // Ransac Plane Estimation
    cout << "_____ ransac plane estimation _____ " << endl;
    cout << "do_use_ransac_plane: " << do_use_ransac_plane << endl;
    cout << "ransac_plane_distance_treshold: " << ransac_plane_distance_treshold << endl;
    cout << "ransac_plane_max_iterations: " << ransac_plane_max_iterations << endl;
    cout << "ransac_plane_probability: " << ransac_plane_probability << endl;
    cout << "ransac_plane_use_refinement: " << ransac_plane_use_refinement << endl;
    cout << "ransac_plane_refinement_treshold: " << ransac_plane_refinement_treshold << endl;
    cout << "ransac_plane_use_camx_treshold: " << ransac_plane_use_camx_treshold << endl;
    cout << "ransac_plane_treshold_camx: " << ransac_plane_treshold_camx << endl;
    cout << "ransac_plane_point_distance_treshold: " << ransac_plane_point_distance_treshold << endl;
    cout << "ransac_plane_debug_visualize: " << ransac_plane_debug_visualize << endl;
    cout << endl;

    // Plane Depth Estimation
    cout << "_____ Plane Depth Estimation ____ " << endl;
    cout << "plane_estimator_use_triangle_maximation: " << plane_estimator_use_triangle_maximation << endl;
    cout << "plane_estimator_z_x_min_relation: " << plane_estimator_z_x_min_relation << endl;
    cout << "plane_estimator_use_leastsquares: " << plane_estimator_use_leastsquares << endl;
    cout << "plane_estimator_use_mestimator: " << plane_estimator_use_mestimator << endl;

    // Misc
    cout << "_____ misc _____ " << endl;
    cout << "do_use_cut_behind_camera: " << do_use_cut_behind_camera << endl;
    cout << "do_use_triangle_size_maximation: " << do_use_triangle_size_maximation << endl;
    cout << "do_check_triangleplanar_condition: " << do_check_triangleplanar_condition << endl;
    cout << "triangleplanar_crossnorm_treshold: " << triangleplanar_crossnorm_treshold << endl;
    cout << "viewray_plane_orthoganality_treshold: " << viewray_plane_orthoganality_treshold << endl;
    cout << "set_all_depths_to_zero: " << set_all_depths_to_zero << endl;
    cout << endl;

    // Debug
    cout << "do_debug_singleFeatures: " << do_debug_singleFeatures << endl;
    cout << "do_publish_points: " << do_publish_points << endl;
    cout << "do_depth_calc_statistics: " << do_depth_calc_statistics << endl;
    cout << endl;
}

} // namespace Img_Lidar
