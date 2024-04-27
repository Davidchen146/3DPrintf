#include "meshoperations.h"

void MeshOperations::getBoundaryFaces(const std::unordered_set<int> &patch_one, const std::unordered_set<int> &patch_two, std::unordered_set<int> &boundaryFaces) {
    std::unordered_set<std::pair<int, int>, PairHash> boundaryEdges;
    getBoundaryEdges(patch_one, patch_two, boundaryEdges);
    // We have the boundary edges, now let's check which faces share those edges
    for (int f: patch_one) {
        updateFaceBoundarySet(boundaryEdges, boundaryFaces, f);
    }
    for (int f: patch_two) {
        updateFaceBoundarySet(boundaryEdges, boundaryFaces, f);
    }
}

void MeshOperations::updateFaceBoundarySet(std::unordered_set<std::pair<int, int>, PairHash> &boundaryEdges, std::unordered_set<int> &boundaryFaces, int face) {
    Vector3i vertex_indices = _faces[face];
    std::pair<int, int> e1 = _mesh.getSortedPair(vertex_indices[0], vertex_indices[1]);
    std::pair<int, int> e2 = _mesh.getSortedPair(vertex_indices[0], vertex_indices[2]);
    std::pair<int, int> e3 = _mesh.getSortedPair(vertex_indices[1], vertex_indices[2]);
    // if any of the face's edges are in the boundary, add face to boundaryFaces
    if (boundaryEdges.contains(e1) || boundaryEdges.contains(e2) || boundaryEdges.contains(e3)) {
        boundaryFaces.insert(face);
    }
}

void MeshOperations::updateFuzzyRegion(std::unordered_set<int> &fuzzyRegion, std::unordered_set<int> &boundaryFaces, int f) {
    double minDistance = std::numeric_limits<double>::max();
    for (int b_f: boundaryFaces) {
        minDistance = std::min(minDistance, getGeodesicDistance(f, b_f));
    }
    if (minDistance <= _fuzzy_region_width) {
        fuzzyRegion.insert(f);
    }
}

void MeshOperations::getFuzzyRegion(const std::unordered_set<int> &patch_one, const std::unordered_set<int> &patch_two, std::unordered_set<int> &fuzzyRegion) {
    // A fuzzy region is a set of faces, we are storing all fuzzy regions in a vector
    unordered_set<int> boundaryFaces;
    getBoundaryFaces(patch_one, patch_two, boundaryFaces);
    fuzzyRegion.insert(boundaryFaces.begin(), boundaryFaces.end());
    for (int f: patch_one) {

    }
    for (int f: patch_two) {

    }
}
