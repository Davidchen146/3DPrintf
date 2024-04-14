#include "meshoperations.h"
#include <igl/blue_noise.h>

// Implments the Oversegmentation Routine and any necessary subroutines

// Generates oversegmentation
void MeshOperations::generateOversegmentation(std::vector<std::vector<int>> &patches) {

}

void MeshOperations::sampleRandomFaces(std::vector<int> &faces, int n) {
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

    // std::cout << FI.size() << std::endl;

    for (int i = 0; i < FI.size(); i++) {
        faces.push_back(FI[i]);
    }
}

void MeshOperations::generateInitialSeeds(std::vector<int> &seeds) {

}

// Iterative steps to generate oversegmentation
// Given seeds, generate patches by assigning faces to a patch
void MeshOperations::generatePatches(const std::vector<int> &seeds, std::vector<std::vector<int>> &patches) {
    // Initialize patches to contain a list for each seed
    patches.clear();
    for (const int &seed : seeds) {
        std::vector<int> patch;
        patches.push_back(patch);
    }

    // Assign faces based on the shortest weighted distance to a seed
    // TODO: Use std::thread to make this faster by parallelizing the computation
    for (int face = 0; face < _faces.size(); face++) {
        std::pair<double, int> min_weighted_distance = getWeightedDistanceToSet(face, seeds, true);
        assert(min_weighted_distance.first < std::numeric_limits<double>::max());
        // Assign this face to the patch it has the smallest distance to
        patches[min_weighted_distance.second].push_back(face);
    }
}

// Given patches, determine the new seed face for that patch
// Seed faces are computed using the weighted average of face's centroids
void MeshOperations::recenterSeeds(const std::vector<std::vector<int>> &patches, std::vector<int> &new_seeds) {
    // Initializes new_seeds to contain the new indices for seed faces
    new_seeds.clear();
    new_seeds.resize(patches.size());

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

        for (const int &face : patches[patch]) {
            Eigen::Vector3f face_centroid = getCentroid(face);
            double distance_to_center = (face_centroid - patch_center).norm();
        }

        assert(closest_face != -1);
        new_seeds[patch] = closest_face;
    }
}
