#include "meshoperations.h"

// Subroutines for Initial Segmentation and implementation of Initial Segmentation

// Initial segmentation: returns list of lists of faces and printing directions
// Takes in an initial list of lists representing patches from oversegmentation
// Each element in the output list represents a printable component
// The other list stores the corresponding printing direction (the ith component is printed in direction i)
void MeshOperations::generateInitialSegmentation(const std::vector<std::unordered_set<int>> &patches,
                                                 std::vector<std::unordered_set<int>> &printable_components,
                                                 std::vector<Eigen::Vector3f> &printing_directions) {
    // goal: populate printable_components, printing_directions

    // Step 1: get printing directions to work with, d
    std::vector<Eigen::Vector3f> directions(_num_random_dir_samples);
    sampleRandomDirections(directions);

    // Step 2: Establish data structures to work with
    // these populate _supportCoefficients, _smoothingCoefficients respectively
    populateSupportMatrix(patches, directions);
    populateSmoothingMatrix();
}

// Subroutines used for Phase 2 (Initial Segmentation)
// Sample random directions
void MeshOperations::sampleRandomDirections(std::vector<Eigen::Vector3f> &directions) {
    directions.clear();

    for (int i = 0; i < _num_random_dir_samples; i++) {
        // randomly sample from the sphere
        float phi = 2.f * std::numbers::pi * (float) rand() / (float) UINT32_MAX;
        float theta = acos(2 * ((float) rand() / (float) UINT32_MAX) - 1);

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

    // No supports on inside of shell (normals should be pointing towards printing direction)
    double dot = faceNormal.dot(direction);
    if (dot > 0) {
        return false;
    }

    // The face needs support if its angle is more than 90 degrees + printing tolerance angle away from the direction
    double angle = acos(dot) / (faceNormal.norm() * direction.norm());

    // compare with tolerance angle + right angle (90 degrees or 1/2 pi)
    return angle > _printer_tolerance_angle + (std::numbers::pi / 2);
}

// Determine if an edge is overhanging and requires support
bool MeshOperations::isEdgeOverhanging(const std::pair<int, int> &edge, const Eigen::Vector3f &direction) {
    Eigen::Vector3f edgeNormal = getEdgeNormal(edge);

    // No supports on inside of shell (normals should be pointing towards printing direction)
    double dot = edgeNormal.dot(direction);
    if (dot > 0) {
        return false;
    }

    // The edge needs support if its angle is more than 90 degrees + printing tolerance angle away from the direction
    double angle = acos(dot) / (edgeNormal.norm() * direction.norm());

    // compare with tolerance angle + right angle (90 degrees or 1/2 pi)
    return angle > _printer_tolerance_angle + (std::numbers::pi / 2);
}

// Determine if a vertex is overhanging and requires support
bool MeshOperations::isVertexOverhanging(const int vertex, const Eigen::Vector3f &direction) {
    Eigen::Vector3f vertexNormal = getVertexNormal(vertex);

    // No supports on inside of shell (normals should be pointing towards printing direction)
    double dot = vertexNormal.dot(direction);
    if (dot > 0) {
        return false;
    }

    // The vertex needs support if its angle is more than 90 degrees + printing tolerance angle away from the direction
    double angle = acos(dot) / (vertexNormal.norm() * direction.norm());

    // compare with tolerance angle + right angle (90 degrees or 1/2 pi)
    return angle > _printer_tolerance_angle + (std::numbers::pi / 2);
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
    // Area * ambient occlusion (exponentiated)
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

void MeshOperations::populateSupportMatrix(const std::vector<std::unordered_set<int>> &patches,
                                           std::vector<Eigen::Vector3f> &directions) {
    // Goal: populate the PxD _supportCoefficients matrix with coefficients, w_ij
    // let P (N in paper) be size of set of patches
    // let D (h in paper) be size of set of directions

    _supportCoefficients.resize(patches.size(), directions.size());
    _supportCoefficients.setZero();

    // this coefficient is area of faces needing support, weighted by ambient occlusion

    // following notations from paper
    for (Eigen::Vector3f i : directions) {
        for (std::unordered_set<int> j : patches) {
            // Goal: sum the cost function over the faces in this patch
            float costSum = 0.0f;
            for (int f : j) { // over the faces

                // Todo: determine whether face requires support and needs to incur a cost

                // case a: face normal has angle w/ base greater than a threshold (paper uses 55)
                if (isFaceOverhanging(f, i)) continue;
                // get the edges to check each of them for overhang
                Face* face = _mesh.getFaceMap().at(f);
                int v1 = face->halfedge->source->index;
                int v2 = face->halfedge->next->source->index;
                int v3 = face->halfedge->next->next->source->index;
                if (isVertexOverhanging(v1, i) || isVertexOverhanging(v2, i) || isVertexOverhanging(v3, i)) continue;
                // establish pairs in ascending order to ensure compatibility
                std::pair<int, int> e1 = (v1 < v2) ? std::make_pair(v1, v2) : std::make_pair(v2, v1);
                std::pair<int, int> e2 = (v2 < v3) ? std::make_pair(v2, v3) : std::make_pair(v3, v2);
                std::pair<int, int> e3 = (v3 < v1) ? std::make_pair(v3, v1) : std::make_pair(v1, v3);
                if (isEdgeOverhanging(e1, i) || isEdgeOverhanging(e2, i) || isEdgeOverhanging(e3, i)) continue;




            }




        }


    }


}

void MeshOperations::populateSmoothingMatrix() {
    return;
}
