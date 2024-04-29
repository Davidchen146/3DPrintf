#include "src/meshoperations.h"
#include <igl/opengl/glfw/Viewer.h>

MeshOperations::MeshOperations(Mesh m) {
    _mesh = m;

    // Set default parameters for operations here
    // Preprocessing parameters
    _geodesic_distance_weight = 0.1;
    _convex_coeff = 0.05;
    _concave_coeff = 1;
    _use_zero_cost_faces = false;
    _solver = nullptr;

    // Oversegmentation parameters
    _num_seed_faces = 0;
    _proportion_seed_faces = 0.1;
    _oversegmentation_bounding_box_coeff = 0.01;
    _num_oversegmentation_iterations = 3;
    _visualize_seeds = false;

    // Initial segmentation parameters
    _num_random_dir_samples = 512;
    _printer_tolerance_angle = 55.f / 180.f * std::numbers::pi; // 55 degrees in radians
    _ambient_occlusion_supports_alpha = 0.5;
    _ambient_occlusion_smoothing_alpha = 0.5;
    _smoothing_width_t = 0.3;
    _ambient_occlusion_samples = 500;
    _footing_samples = 1;
    _axis_only = false;
}

// Configure parameters for 3D printing operations
void MeshOperations::setPreprocessingParameters(double geodesic_weight,
                                                double convex_coeff,
                                                double concave_coeff,
                                                bool use_zero_cost_faces) {
    // Check against default value (0) to prevent loading in unspecified parameters
    if (geodesic_weight != 0.0) {
        _geodesic_distance_weight = geodesic_weight;
    }

    if (convex_coeff != 0.0) {
        _convex_coeff = convex_coeff;
    }

    if (concave_coeff != 0.0) {
        _concave_coeff = concave_coeff;
    }

    if (use_zero_cost_faces) {
        _use_zero_cost_faces = true;
    }
}

void MeshOperations::setOversegmentationParameters(int num_seed_faces,
                                                   double proportion_seed_faces,
                                                   double bounding_box_coeff,
                                                   int num_iterations,
                                                   bool visualize_seeds) {
    // Check against default value (0) to prevent loading in unspecified parameters
    if (num_seed_faces != 0.0) {
        _num_seed_faces = num_seed_faces;
    }

    if (proportion_seed_faces != 0.0) {
        _proportion_seed_faces = proportion_seed_faces;
    }

    if (bounding_box_coeff != 0.0) {
        _oversegmentation_bounding_box_coeff = bounding_box_coeff;
    }

    if (num_iterations != 0) {
        _num_oversegmentation_iterations = num_iterations;
    }

    if (visualize_seeds) {
        _visualize_seeds = visualize_seeds;
    }
}

void MeshOperations::setInitialSegmentationParameters(int num_random_dir_samples,
                                                      double printer_tolerance_angle,
                                                      double ambient_occlusion_supports_alpha,
                                                      double ambient_occlusion_smoothing_alpha,
                                                      double smoothing_width_t,
                                                      int ambient_occlusion_samples,
                                                      int footing_samples,
                                                      bool axis_only) {
    // Check against default value (0) to prevent loading in unspecified parameters
    if (num_random_dir_samples != 0) {
        _num_random_dir_samples = num_random_dir_samples;
    }

    if (printer_tolerance_angle != 0.0) {
        // Convert the parameter to radians
        double angle_rads = printer_tolerance_angle / 180.f * std::numbers::pi;
        _printer_tolerance_angle = angle_rads;
    }

    if (ambient_occlusion_supports_alpha != 0.0) {
        _ambient_occlusion_supports_alpha = ambient_occlusion_supports_alpha;
    }

    if (ambient_occlusion_smoothing_alpha != 0.0) {
        _ambient_occlusion_smoothing_alpha = ambient_occlusion_smoothing_alpha;
    }

    if (smoothing_width_t != 0.0) {
        _smoothing_width_t = smoothing_width_t;
    }

    if (ambient_occlusion_samples != 0) {
        _ambient_occlusion_samples = ambient_occlusion_samples;
    }

    if (footing_samples != 0) {
        _footing_samples = footing_samples;
    }

    // Only generate a printing direction for each axis
    if (axis_only) {
        _axis_only = true;
        _num_random_dir_samples = 6;
    }
}

double MeshOperations::getGeodesicDistance(int i, int j) {
    return _geodesicDistances(i, j);
}

double MeshOperations::getWeightedDistance(int i, int j) {
    return _weightedDistances(i, j);
}

double MeshOperations::getAngularDistance(int i, int j) {
    return _angularDistances(i, j);
}
