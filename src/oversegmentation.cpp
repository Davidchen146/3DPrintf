#include "src/meshoperations.h"
#include <igl/blue_noise.h>

// Implments the Oversegmentation Routine and any necessary subroutines

// Generates oversegmentation
void MeshOperations::generateOversegmentation(std::vector<std::unordered_set<int>> &patches) {
    // Init the patches vec
    patches.clear();

    // Generate the initial seeds
    std::unordered_set<int> seeds;
    generateInitialSeeds(seeds);

    for (int num_iterations = 0; num_iterations < _num_oversegmentation_iterations; num_iterations++) {
        // Grow from the seeds to get the patches
        generatePatches(seeds, patches);
        // Recenter seeds based on the patches
        if (num_iterations < _num_oversegmentation_iterations - 1) {
            recenterSeeds(patches, seeds);
        }
    }

    // At this point, the patches should be generated
    // If we only want the seeds, set patches to only consist of the seeds
    if(_seeds_only) {
        auto current_seed = seeds.begin();
        for (int seednum = 0; seednum < seeds.size(); seednum++) {
            patches[seednum].clear();
            patches[seednum].insert(*current_seed);
            current_seed++;
        }
    }
}

void MeshOperations::sampleRandomFaces(std::unordered_set<int> &faces, int n) {
    // n is *approximately* the number of faces you want to sample
    // but I can't get it such that it's always exactly the number of faces you want to sample
    // this is just a heuristic so modify however you wish
    Eigen::VectorXd A;
    igl::doublearea(_V, _F, A);
    double r = sqrt(A.sum()/(n * igl::PI));

    Eigen::MatrixXd B;
    Eigen::VectorXi FI; // indices into _F
    Eigen::MatrixXd P_blue;
    igl::blue_noise(_V,_F,r,B,FI,P_blue);

    std::cout << "Faces sampled: " << FI.size() << std::endl;

    for (int i = 0; i < FI.size(); i++) {
        faces.insert(FI[i]);
    }
}

void MeshOperations::generateInitialSeeds(std::unordered_set<int> &seeds) {
    // Initialize seeds vector
    seeds.clear();

    int num_samples = _num_seed_faces;
    // Prioritize hard-coded seed face number, use proportion as heuristic if not present
    if (num_samples == 0) {
        num_samples = std::round(_proportion_seed_faces * numFaces);
    }
    std::unordered_set<int> seed_candidates;
    sampleRandomFaces(seed_candidates, num_samples);

    std::cout << "Number of seed candidates " << seed_candidates.size() << std::endl;

    // Determine the initial central face by selecting the face with the smallest sum of distances to all other faces
    int center_face = -1;
    double center_min_distance = std::numeric_limits<double>::max();
    for (auto seed_candidate = seed_candidates.begin(); seed_candidate != seed_candidates.end(); seed_candidate++) {
        double seed_sum_distance = getTotalGeodesicDistanceToSet(*seed_candidate, seed_candidates);

        // Update minimal distance
        if (seed_sum_distance < center_min_distance) {
            center_min_distance = seed_sum_distance;
            center_face = *seed_candidate;
        }
    }

    // This becomes the center face
    assert(center_face != -1);
    seeds.insert(center_face);
    // Remove it from the list of candidate seeds
    seed_candidates.erase(center_face);

    // Continue to add faces until the maximum distance between any pairs of faces is sufficiently small
    double maximum_distance = std::numeric_limits<double>::max();
    // Threshold for convergence is a multiple of the bounding box diagonal
    int iteration = 0;
    while(maximum_distance > bbd * _oversegmentation_bounding_box_coeff && seed_candidates.size() > 0) {
        // Pick a face with the largest geodesic distance to the set of seeds
        double largest_distance = -1;
        int next_face = -1;
        for (auto seed_candidate = seed_candidates.begin(); seed_candidate != seed_candidates.end(); seed_candidate++) {
            double dist_to_seeds = getMinGeodesicDistanceToSet(*seed_candidate, seeds).first;
            if (dist_to_seeds > largest_distance) {
                largest_distance = dist_to_seeds;
                next_face = *seed_candidate;
            }
        }

        // This is the new seed face
        seeds.insert(next_face);
        // Remove it from the candidate seeds
        seed_candidates.erase(next_face);

        // Determine convergence by computing distances between seeds
        maximum_distance = -1;
        for (auto seed = seeds.begin(); seed != seeds.end(); seed++) {
            double seed_dist_to_other_seeds = getMinGeodesicDistanceToSet(*seed, seeds).first;
            if (seed_dist_to_other_seeds > maximum_distance) {
                maximum_distance = seed_dist_to_other_seeds;
            }
        }

        assert(maximum_distance != -1);
        // If we iterate again, we have added another seed and we have chosen to continue
        iteration++;
    }
    // If we've reached this point, we've terminated with our set of seeds
    std::cout << "Number of selected final seeds " << seeds.size() << std::endl;
}

// Iterative steps to generate oversegmentation
// Given seeds, generate patches by assigning faces to a patch
void MeshOperations::generatePatches(const std::unordered_set<int> &seeds, std::vector<std::unordered_set<int>> &patches) {
    // Initialize patches to contain a list for each seed
    patches.clear();
    std::unordered_map<int, int> numToIndex;
    int index = 0;
    for (const int &seed : seeds) {
        std::unordered_set<int> patch;
        patches.push_back(patch);
        numToIndex[seed] = index;
        index++;
    }


    // Assign faces based on the shortest weighted distance to a seed
    // TODO: Use std::thread to make this faster by parallelizing the computation
    for (int face = 0; face < _faces.size(); face++) {
        std::pair<double, int> min_weighted_distance = getMinWeightedDistanceToSet(face, seeds, true);
        assert(min_weighted_distance.first < std::numeric_limits<double>::max());
        // Assign this face to the patch it has the smallest distance to
        patches[numToIndex[min_weighted_distance.second]].insert(face);
    }
}

// Given patches, determine the new seed face for that patch
// Seed faces are computed using the weighted average of face's centroids
void MeshOperations::recenterSeeds(const std::vector<std::unordered_set<int>> &patches, std::unordered_set<int> &new_seeds) {
    assert(new_seeds.size() == patches.size());
    // Initializes new_seeds to contain the new indices for seed faces
    new_seeds.clear();

    // For each patch, compute its center by weighting the centroid of each face by its area
    // TODO: Use std::thread to make this faster by parallelizing the computation
    for (int patch = 0; patch < patches.size(); patch++) {
        double patch_area = 0.0;
        Eigen::Vector3f patch_center(0.0f, 0.0f, 0.0f);

        for (const int &face : patches[patch]) {
            double face_area = getArea(face);
            patch_area += face_area;
            patch_center += face_area * getCentroid(face);
        }

        // Adjust centroid by weighting
        patch_center /= patch_area;

        // The new seed is the patch whose centroid is closest to the weighted center of the patch
        int closest_face = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (const int &face : patches[patch]) {
            Eigen::Vector3f face_centroid = getCentroid(face);
            double distance_to_center = (face_centroid - patch_center).norm();
            // This is the face whose centroid is closest to the center of the patch
            if (distance_to_center < min_dist) {
                closest_face = face;
            }
        }

        assert(closest_face != -1);
        new_seeds.insert(closest_face);
    }
    assert(new_seeds.size() == patches.size());
}
