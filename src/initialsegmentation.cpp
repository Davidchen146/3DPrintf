#include "meshoperations.h"

// Subroutines for Initial Segmentation and implementation of Initial Segmentation

// Initial segmentation: returns list of lists of faces and printing directions
// Takes in an initial list of lists representing patches from oversegmentation
// Each element in the output list represents a printable component
// The other list stores the corresponding printing direction (the ith component is printed in direction i)
void MeshOperations::generateInitialSegmentation(const std::vector<std::unordered_set<int>> &patches,
                                                 std::vector<std::unordered_set<int>> &printable_components,
                                                 std::vector<Eigen::Vector3f> &printing_directions) {
    return;
}

// Subroutines used for Phase 2 (Initial Segmentation)
// Sample random directions
void MeshOperations::sampleRandomDirections(std::vector<Eigen::Vector3f> &directions) {
    for (int i = 0; i < _num_random_dir_samples; i++) {
        // randomly sample from the sphere
        float phi = 2.f * M_PI * (float) arc4random() / (float) UINT32_MAX;
        float theta = acos(2 * ((float) arc4random() / (float) UINT32_MAX) - 1);

        // spherical to rectangular converion (with radius = 1)
        float x = sin(theta) * cos(phi);
        float y = cos(theta);
        float z = sin(theta) * sin(phi);
        Vector3f direction(x, y, z);

        // NOTE: assumes that directions starts off as an empty vector
        directions.push_back(direction);
    }
}

// Determine if a face should be supported
// Note: this might need to be computed for all faces in a patch at once; not sure
bool MeshOperations::isFaceSupported(const int face, const Eigen::Vector3f &direction, const std::vector<std::unordered_set<int>> &patches) {
    return false;
}

// Determine if a face is overhanging and requires support
bool MeshOperations::isFaceOverhanging(const int face, const Eigen::Vector3f &direction) {
    Eigen::Vector3f faceNormal = getFaceNormal(face);
    // get angle between face normal and printing direction (in degrees)
    double angle = acos(faceNormal.dot(direction) / (faceNormal.norm() * direction.norm())) * 180 / M_PI;
    // compare with tolerance angle
    return angle > _printer_tolerance_angle;
}

// Determine if an edge is overhanging and requires support
bool MeshOperations::isEdgeOverhanging(const std::pair<int, int> &edge, const Eigen::Vector3f &direction) {
    Eigen::Vector3f edgeNormal = getEdgeNormal(edge);
    // get angle between edge normal and printing direction (in degrees)
    double angle = acos(edgeNormal.dot(direction) / (edgeNormal.norm() * direction.norm())) * 180 / M_PI;
    // compare with tolerance angle
    return angle > _printer_tolerance_angle;
}

// Determine if a vertex is overhanging and requires support
bool MeshOperations::isVertexOverhanging(const int vertex, const Eigen::Vector3f &direction) {
    Eigen::Vector3f vertexNormal = getVertexNormal(vertex);
    // get angle between vertex normal and printing direction (in degrees)
    double angle = acos(vertexNormal.dot(direction) / (vertexNormal.norm() * direction.norm())) * 180 / M_PI;
    // compare with tolerance angle
    return angle > _printer_tolerance_angle;
}

// Determine if a face is footing a supported face and requires support
// Note: this function might not have enough parameters to complete its implementation; not sure
// It may need to keep track of what faces have already been supported and what patches they belong to
bool MeshOperations::isFaceFooted(const int face, const Eigen::Vector3f &direction, const std::vector<std::unordered_set<int>> &patches) {
    return false;
}

// Compute support coefficient for a face in direction
double MeshOperations::computeSupportCoefficient(const int face, const Eigen::Vector3f &direction,
                                                 const std::vector<std::unordered_set<int>> &patches) {
    return 0.0;
}

// Compute smoothing coefficient between two sets of faces
double MeshOperations::computeSmoothingCoefficient(const std::vector<std::unordered_set<int>> &patch_one,
                                                   const std::vector<std::unordered_set<int>> &patch_two) {
    return 0.0;
}

// Interface to ILP
void MeshOperations::assignPrintingDirections(const std::vector<std::vector<int>> &patches,
                                              const std::vector<Eigen::Vector3f> &printing_directions,
                                              std::vector<Eigen::Vector3f> &patch_printing_directions) {
    return;
}

// Assign results of the ILP to something we can return out
void MeshOperations::generatePrintableComponents(const std::vector<std::vector<int>> &patches,
                                                 std::vector<unordered_set<int>> &printable_components,
                                                 const std::vector<Eigen::Vector3f> &patch_printing_directions,
                                                 std::vector<Eigen::Vector3f> &component_printing_directions) {
    return;
}
