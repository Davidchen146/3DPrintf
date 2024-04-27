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
    // the fuzzy region starts by including all boundary faces
    // fuzzyRegion.insert(boundaryFaces.begin(), boundaryFaces.end());
    for (int f: patch_one) {
        updateFuzzyRegion(fuzzyRegion, boundaryFaces, f);
    }
    for (int f: patch_two) {
        updateFuzzyRegion(fuzzyRegion, boundaryFaces, f);
    }
}

void MeshOperations::generateFuzzyRegions(std::vector<std::unordered_set<int>> &printable_components, std::vector<FuzzyNode*> &nodes) {
    // generate pairwise fuzzy regions
    for (int i = 0; i < printable_components.size(); i++) {
        for (int j = i+1; j < printable_components.size(); j++) {
            // pointer to fuzzyRegion to avoid the struct having to copy everything over on initialization
            unordered_set<int>* fuzzyRegion = new unordered_set<int>();
            unordered_set<FuzzyNode*> neighbors;
            getFuzzyRegion(printable_components[i], printable_components[j], *fuzzyRegion);
            FuzzyNode* fuzzyNode = new FuzzyNode(fuzzyRegion, neighbors);
            nodes.push_back(fuzzyNode);
        }
    }
}

bool areNodesConnected(FuzzyNode* n1, FuzzyNode* n2) {
    unordered_set<int>* n1_faces = n1->fuzzyRegion;
    unordered_set<int>* n2_faces = n2->fuzzyRegion;
    for (int i: *n1_faces) {
        if (n2_faces->contains(i)) {
            return true;
        }
    }
    return false;
}

void MeshOperations::makeFuzzyGraph(std::vector<FuzzyNode*> &nodes) {
    for (int i = 0; i < nodes.size(); i++) {
        for (int j = i+1; j < nodes.size(); j++) {
            FuzzyNode* n1 = nodes[i];
            FuzzyNode* n2 = nodes[j];

            if (areNodesConnected(n1, n2)) {
                n1->neighbors.insert(n2);
                n2->neighbors.insert(n1);
            }
        }
    }
}

void fuzzyDFS(unordered_set<FuzzyNode*> &visited, unordered_set<int> &regionFaces, FuzzyNode* node) {
    if (visited.contains(node)) {
        return;
    }
    // Add the node's faces to the overall fuzzy region
    for (int i: *node->fuzzyRegion) {
        regionFaces.insert(i);
    }
    // Mark node as visited
    visited.insert(node);
    // Visit node neighbors
    for (FuzzyNode* n: node->neighbors) {
        fuzzyDFS(visited, regionFaces, n);
    }
}

void MeshOperations::combineFuzzyRegions(std::vector<FuzzyNode*> &nodes, std::vector<std::unordered_set<int>> &fuzzyRegions) {
    for (FuzzyNode* node: nodes) {
        unordered_set<FuzzyNode*> visited;
        unordered_set<int> fuzzyRegion;
        fuzzyDFS(visited, fuzzyRegion, node);
        fuzzyRegions.push_back(fuzzyRegion);
    }
}


