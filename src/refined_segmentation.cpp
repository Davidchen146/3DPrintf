#include "meshoperations.h"

void MeshOperations::getBoundaryFaces(const std::unordered_set<int> &patch_one, const std::unordered_set<int> &patch_two, std::unordered_set<int> &boundaryFaces) {
    std::unordered_set<std::pair<int, int>, PairHash> boundaryEdges;
    getBoundaryEdges(patch_one, patch_two, boundaryEdges);
    unordered_set<int> edgeVertices;
    for (pair<int, int> edge: boundaryEdges) {
        edgeVertices.insert(edge.first);
        edgeVertices.insert(edge.second);
    }
    // We have the boundary edges, now let's check which faces are adjacent to those edges
    for (int f: patch_one) {
        Vector3i vertex_indices = _faces[f];
        // if any of the face's vertices are in the boundary, add face to boundaryFaces
        if (edgeVertices.contains(vertex_indices[0]) || edgeVertices.contains(vertex_indices[1]) || edgeVertices.contains(vertex_indices[2])) {
            boundaryFaces.insert(f);
        }
    }
    for (int f: patch_two) {
        Vector3i vertex_indices = _faces[f];
        // if any of the face's vertices are in the boundary, add face to boundaryFaces
        if (edgeVertices.contains(vertex_indices[0]) || edgeVertices.contains(vertex_indices[1]) || edgeVertices.contains(vertex_indices[2])) {
            boundaryFaces.insert(f);
        }
    }
}

// Given a face, see if it belongs to the given fuzzy region
void MeshOperations::updateFuzzyRegion(std::unordered_set<int> &fuzzyRegion, std::unordered_set<int> &boundaryFaces, int f) {
    double minDistance = std::numeric_limits<double>::max();
    for (int b_f: boundaryFaces) {
        minDistance = std::min(minDistance, getGeodesicDistance(f, b_f));
    }
    if (minDistance <= _fuzzy_region_width) {
        fuzzyRegion.insert(f);
    }
}

void MeshOperations::getPairwiseFuzzyRegion(const std::unordered_set<int> &patch_one, const std::unordered_set<int> &patch_two, std::unordered_set<int> &fuzzyRegion) {
    // A fuzzy region is a set of faces, we are storing all fuzzy regions in a vector
    unordered_set<int> boundaryFaces;
    getBoundaryFaces(patch_one, patch_two, boundaryFaces);
    // the fuzzy region starts by including all boundary faces
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
            getPairwiseFuzzyRegion(printable_components[i], printable_components[j], *fuzzyRegion);
            if (fuzzyRegion->size() == 0) {
                // If the pairwise patches don't share boundaries
                delete fuzzyRegion;
            } else {
                FuzzyNode* fuzzyNode = new FuzzyNode{fuzzyRegion, neighbors};
                nodes.push_back(fuzzyNode);
            }
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
    unordered_set<FuzzyNode*> visited;
    for (FuzzyNode* node: nodes) {
        if (visited.contains(node)) {
            continue;
        }
        unordered_set<int> fuzzyRegion;
        fuzzyDFS(visited, fuzzyRegion, node);
        fuzzyRegions.push_back(fuzzyRegion);
    }
}

void MeshOperations::generateRefinedSegmentation(std::vector<std::unordered_set<int>> &printable_components, std::vector<std::unordered_set<int>> &fuzzyRegions) {
    vector<FuzzyNode*> nodes;
    std::cout << "Making initial fuzzy regions..." << std::endl;
    generateFuzzyRegions(printable_components, nodes);
    std::cout << "Number of initial fuzzy regions: " << nodes.size() << std::endl;
    makeFuzzyGraph(nodes);
    combineFuzzyRegions(nodes, fuzzyRegions);
    std::cout << "Number of total fuzzy regions: " << fuzzyRegions.size() << std::endl;

    std::cout << "Size of Fuzzy Region 1: " << fuzzyRegions[0].size() <<  std::endl;
    // for (int i: fuzzyRegions[0]) {
    //     std::cout << i << std::endl;
    // }
    std::cout << "Size of Fuzzy Region 2: " << fuzzyRegions[1].size() << std::endl;
    // for (int i: fuzzyRegions[1]) {
    //     std::cout << i << std::endl;
    // }
}


