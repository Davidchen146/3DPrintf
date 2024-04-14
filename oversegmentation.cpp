#include "meshoperations.h"

// Implments the Oversegmentation Routine and any necessary subroutines

// Generates oversegmentation
std::vector<std::vector<int>> MeshOperations::generateOversegmentation(){

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

}
