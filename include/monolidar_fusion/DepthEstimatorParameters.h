#pragma once

#include <string>

namespace Mono_Lidar {

class DepthEstimatorParameters {
public:
    void fromFile(const std::string& filePath);
    void print();

    // Nearest Neighbors search
    bool do_use_nearestNeighborSearch{false};
    int nnSearch_count{10};

    std::string str_do_use_nearestNeighborSearch = "do_use_nearestNeighborSearch";
    std::string str_nnSearch_count = "nnSearch_count";

    // Neighbor search
    int neighbor_search_mode{0};
    int pixelarea_search_witdh{12};
    int pixelarea_search_height{15};
    int pixelarea_search_offset_x{0};
    int pixelarea_search_offset_y{0};

    std::string str_neighbor_search_mode = "neighbor_search_mode";
    std::string str_pixelarea_search_witdh = "pixelarea_search_witdh";
    std::string str_pixelarea_search_height = "pixelarea_search_height";
    std::string str_pixelarea_search_offset_x = "pixelarea_search_offset_x";
    std::string str_pixelarea_search_offset_y = "pixelarea_search_offset_y";

    // Radius search
    bool do_use_radiusSearch{true};
    double radiusSearch_radius{10};
    int radiusSearch_count_min{3};

    std::string str_do_use_radiusSearch = "do_use_radiusSearch";
    std::string str_radiusSearch_radius = "radiusSearch_radius";
    std::string str_radiusSearch_count_min = "radiusSearch_count_min";

    // Histogram Segmentation
    bool do_use_histogram_segmentation{true};
    double histogram_segmentation_bin_witdh{0.5};
    int histogram_segmentation_min_pointcount{3};

    std::string str_do_use_histogram_segmentation = "do_use_histogram_segmentation";
    std::string str_histogram_segmentation_bin_witdh = "histogram_segmentation_bin_witdh";
    std::string str_histogram_segmentation_min_pointcount = "histogram_segmentation_min_pointcount";

    // Depth segmentation
    bool do_use_depth_segmentation{0};
    double depth_segmentation_max_treshold_gradient{0.0};
    double depth_segmentation_max_neighbor_distance{0.3};
    double depth_segmentation_max_neighbor_distance_gradient{0.0};
    double depth_segmentation_max_neighbor_to_seedpoint_distance{0.5};
    double depth_segmentation_max_seedpoint_to_seedpoint_distance_gradient{0.0};
    double depth_segmentation_max_seedpoint_to_seedpoint_distance{0.5};
    double depth_segmentation_max_neighbor_to_seedpoint_distance_gradient{0.0};

    double depth_segmentation_max_pointcount{5};

    std::string str_do_use_depth_segmentation = "do_use_depth_segmentation";
    std::string str_depth_segmentation_max_treshold_gradient = "depth_segmentation_max_treshold_gradient";
    std::string str_depth_segmentation_max_neighbor_distance = "depth_segmentation_max_neighbor_distance";
    std::string str_depth_segmentation_max_neighbor_distance_gradient =
        "depth_segmentation_max_neighbor_distance_gradient";
    std::string str_depth_segmentation_max_neighbor_to_seedpoint_distance =
        "depth_segmentation_max_neighbor_to_seedpoint_distance";
    std::string str_depth_segmentation_max_seedpoint_to_seedpoint_distance_gradient =
        "depth_segmentation_max_seedpoint_to_seedpoint_distance_gradient";
    std::string str_depth_segmentation_max_seedpoint_to_seedpoint_distance =
        "depth_segmentation_max_seedpoint_to_seedpoint_distance";
    std::string str_depth_segmentation_max_neighbor_to_seedpoint_distance_gradient =
        "depth_segmentation_max_neighbor_to_seedpoint_distance_gradient";
    std::string str_depth_segmentation_max_pointcount = "depth_segmentation_max_pointcount";

    // Calculated Depth treshold
    bool treshold_depth_enabled{true};
    int treshold_depth_mode{0};
    int treshold_depth_max{100};
    int treshold_depth_min{0};

    std::string str_treshold_depth_enabled = "treshold_depth_enabled";
    std::string str_treshold_depth_mode = "treshold_depth_mode";
    std::string str_treshold_depth_max = "treshold_depth_max";
    std::string str_treshold_depth_min = "treshold_depth_min";

    // Calculate Deph Treshold local
    bool treshold_depth_local_enabled{true};
    int treshold_depth_local_mode{0};
    int treshold_depth_local_valuetype{1};
    double treshold_depth_local_value{0.5};

    std::string str_treshold_depth_local_enabled = "treshold_depth_local_enabled";
    std::string str_treshold_depth_local_mode = "treshold_depth_local_mode";
    std::string str_treshold_depth_local_valuetype = "treshold_depth_local_valuetype";
    std::string str_treshold_depth_local_value = "treshold_depth_local_value";

    // PCA
    bool do_use_PCA{false};
    bool pca_debug{false};
    double pca_treshold_3_abs_min{0.005};
    double pca_treshold_3_2_rel_max{15};
    double pca_treshold_2_1_rel_min{0.5};

    std::string str_do_use_PCA = "do_use_PCA";
    std::string str_pca_debug = "pca_debug";
    std::string str_pca_treshold_3_abs_min = "pca_treshold_3_abs_min";
    std::string str_pca_treshold_3_2_rel_max = "pca_treshold_3_2_rel_max";
    std::string str_pca_treshold_2_1_rel_min = "pca_treshold_2_1_rel_min";

    // Ransac Plane Estimation
    bool do_use_ransac_plane{true};
    double ransac_plane_distance_treshold{0.2};
    int ransac_plane_max_iterations{10000};
    bool ransac_plane_use_refinement{true};
    double ransac_plane_refinement_treshold{10.2};
    bool ransac_plane_use_camx_treshold{false};
    double ransac_plane_treshold_camx{2.0};
    double ransac_plane_point_distance_treshold{0.2};
    bool ransac_plane_debug_visualize{false};
    double ransac_plane_probability{0.999};

    std::string str_do_use_ransac_plane = "do_use_ransac_plane";
    std::string str_ransac_plane_distance_treshold = "ransac_plane_distance_treshold";
    std::string str_ransac_plane_max_iterations = "ransac_plane_max_iterations";
    std::string str_ransac_plane_use_refinement = "ransac_plane_use_refinement";
    std::string str_ransac_plane_refinement_treshold = "ransac_plane_refinement_treshold";
    std::string str_ransac_plane_use_camx_treshold = "ransac_plane_use_camx_treshold";
    std::string str_ransac_plane_treshold_camx = "ransac_plane_treshold_camx";
    std::string str_ransac_plane_point_distance_treshold = "ransac_plane_point_distance_treshold";
    std::string str_ransac_plane_debug_visualize = "ransac_plane_use_camx_treshold";
    std::string str_ransac_plane_probability = "ransac_plane_probability";

    // _____ plane depth estimation _____
    bool plane_estimator_use_triangle_maximation{false};
    double plane_estimator_z_x_min_relation{0};
    bool plane_estimator_use_leastsquares{false};
    bool plane_estimator_use_mestimator{true};

    std::string str_plane_estimator_use_triangle_maximation = "plane_estimator_use_triangle_maximation";
    std::string str_plane_estimator_z_x_min_relation = "plane_estimator_z_x_min_relation";
    std::string str_plane_estimator_use_leastsquares = "plane_estimator_use_leastsquares";
    std::string str_plane_estimator_use_mestimator = "plane_estimator_use_mestimator";

    // Misc
    bool do_use_cut_behind_camera{true};
    bool do_use_triangle_size_maximation{true};
    bool do_check_triangleplanar_condition{true};
    double triangleplanar_crossnorm_treshold{0.1};
    double viewray_plane_orthoganality_treshold{01};
    bool set_all_depths_to_zero{false};

    std::string str_do_use_cut_behind_camera = "do_use_cut_behind_camera";
    std::string str_do_use_triangle_size_maximation = "do_use_triangle_size_maximation";
    std::string str_do_check_triangleplanar_condition = "do_check_triangleplanar_condition";
    std::string str_triangleplanar_crossnorm_treshold = "triangleplanar_crossnorm_treshold";
    std::string str_viewray_plane_orthoganality_treshold = "viewray_plane_orthoganality_treshold";
    std::string str_set_all_depths_to_zero = "set_all_depths_to_zero";

    // Debug
    bool do_debug_singleFeatures{false};
    bool do_publish_points{true};
    bool do_depth_calc_statistics{true};

    std::string str_do_debug_singleFeatures = "do_debug_singleFeatures";
    std::string str_do_publish_points = "do_publish_points";
    std::string str_do_depth_calc_statistics = "do_depth_calc_statistics";
};

} // namespace Img_Lidar
