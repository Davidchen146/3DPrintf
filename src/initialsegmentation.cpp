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
    populateSmoothingMatrix(patches);

    // OK WE ARE GOING TO DO THE THING
}

// Subroutines used for Phase 2 (Initial Segmentation)
// Sample random directions
void MeshOperations::sampleRandomDirections(std::vector<Eigen::Vector3f> &directions) {
    directions.clear();

    for (int i = 0; i < _num_random_dir_samples; i++) {
        Vector3f direction = generateRandomVector();

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
void MeshOperations::findFootingFaces(const int face, const Eigen::Vector3f &direction, std::vector<int> &footing_faces) {
    footing_faces.clear();

    // Shoot random rays from this face
    for (int ray = 0; ray < _footing_samples; ray++) {
        Eigen::Vector3f ray_origin = sampleRandomPoint(face) + (epsilon * getFaceNormal(face));
        int intersected_face = getIntersection(ray_origin, -direction);

        if (intersected_face >= 0) {
            footing_faces.push_back(intersected_face);
        }
    }
}

// Compute support coefficient for a face in direction
double MeshOperations::computeSupportCoefficient(const int face, const Eigen::Vector3f &direction,
                                                 const std::vector<std::unordered_set<int>> &patches) {
    // Area * ambient occlusion (exponentiated)
    // get area
    double area = getArea(face);
    double faceAO = getFaceAO(face);
    return area * std::pow(faceAO, _ambient_occlusion_smoothing_alpha);
}

// Compute smoothing coefficient between two sets of faces
double MeshOperations::computeSmoothingCoefficient(const std::unordered_set<int> &patch_one,
                                                   const std::unordered_set<int> &patch_two) {
    std::unordered_set<std::pair<int, int>, PairHash> boundaryEdges;
    getBoundaryEdges(patch_one, patch_two, boundaryEdges);
    double weight = 0;
    for (std::pair<int, int> edge: boundaryEdges) {
        Vector3f start = _vertices[edge.first];
        Vector3f end = _vertices[edge.second];
        double length = (start - end).norm();
        double AO = getEdgeAO(edge);
        weight += length * std::pow(AO, _ambient_occlusion_smoothing_alpha);
    }
    return weight * _smoothing_width_t;
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
    //for (Eigen::Vector3f i : directions) {
    for (int dirInd = 0; dirInd < directions.size(); dirInd++) {
        // for (std::unordered_set<int> j : patches) {
        for (int patchInd = 0; patchInd < patches.size(); patchInd++) {
            Eigen::Vector3f dir = directions[dirInd];
            std::unordered_set<int> currPatch = patches[patchInd];
            // Goal: sum the cost function over the faces in this patch
            double costSum = 0.0f;
            std::unordered_set<int> footedFaces;

            for (int f : currPatch) { // over the faces

                // Todo: determine whether face requires support and needs to incur a cost
                bool costNeeded = false;
                // case a: face normal has angle w/ base greater than a threshold (paper uses 55)
                if (isFaceOverhanging(f, dir)) costNeeded = true;
                else {
                    // get the edges to check each of them for overhang
                    Face* face = _mesh.getFaceMap().at(f);
                    int v1 = face->halfedge->source->index;
                    int v2 = face->halfedge->next->source->index;
                    int v3 = face->halfedge->next->next->source->index;
                    std::pair<int, int> e1 = (v1 < v2) ? std::make_pair(v1, v2) : std::make_pair(v2, v1);
                    std::pair<int, int> e2 = (v2 < v3) ? std::make_pair(v2, v3) : std::make_pair(v3, v2);
                    std::pair<int, int> e3 = (v3 < v1) ? std::make_pair(v3, v1) : std::make_pair(v1, v3);
                    // case b, c: edge or vertex normals form  angle w/ base greater than a threshold (paper uses 55)
                    if (isVertexOverhanging(v1, dir) || isVertexOverhanging(v2, dir) || isVertexOverhanging(v3, dir)) costNeeded = true;
                    else if (isEdgeOverhanging(e1, dir) || isEdgeOverhanging(e2, dir) || isEdgeOverhanging(e3, dir)) costNeeded = true;
                }
                // Now, handle getting the cost and seeing if there is another part of the mesh that will
                // have to act as supports now.
                if (costNeeded) {
                    costSum += computeSupportCoefficient(f, dir, patches);
                    std::vector<int> newFootedFaces;
                    findFootingFaces(f, dir, newFootedFaces);
                    for (int newFace : newFootedFaces) footedFaces.insert(newFace);
                }
            }
            // Next -- loop over the footedFaces to have their corresponding matrix entry
            // account for their being used as footing.
            for (int footedFace : footedFaces) {
                costSum += computeSupportCoefficient(footedFace, dir, patches);
            }
            _supportCoefficients(patchInd, dirInd) = costSum;
        }
    }
}

// REMEMBER TO DO SECOND LOOP FOR THE SUPPORTED FACES


void MeshOperations::populateSmoothingMatrix(const std::vector<std::unordered_set<int>> &patches) {
    int numPatches = patches.size();
    for (int i = 0; i < numPatches; i++) {
        for (int j = i+1; j < numPatches; j++) {
            double cost = computeSmoothingCoefficient(patches[i], patches[j]);
            if (cost > 0.f) {
                // The pair here is the pair of PATCH indices, where j > i
                std::pair<int, int> pair = std::make_pair(i, j);
                _smoothingCoefficients[pair] = cost;
            }
        }
    }
}
