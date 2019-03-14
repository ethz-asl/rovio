/*
 * NNParams.h
 *
 *  Created on: Dec 8, 2016
 *      Author: wilczynski
 */

#pragma once

namespace Mono_Lidar {
struct NNParams {
    int search_params = 32;
    int kd_index_tree_params = 8;
    bool kd_radius_search = false;
    float kd_radius = 20;
    int number_nn_kd_search = 3;
    int knn = 3;
    bool use_depth_estimator = true;
    double depht_estimator_ratio = 0.5;
    bool write_norm = false;
    bool show_nn_search = false;
    double scale_to_max_dist = 20;
    double max_dist_to_nn = 0;
    bool show_all_points = true;

    void parameterToString(std::stringstream& para_string) {
        para_string << "--NN params--"
                    << "\n";
        para_string << "search_params: " << search_params << "\n";

        para_string << "kd_index_tree_params: " << kd_index_tree_params << "\n";
        para_string << "knn: " << knn << "\n";
        para_string << "use_depth_estimator: " << use_depth_estimator << "\n";
        para_string << "depht_estimator_ratio: " << depht_estimator_ratio << "\n";
    }
};
}
