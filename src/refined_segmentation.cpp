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

    // Distance set by bounding box diagonal + configurable coefficient
    if (minDistance <= bbd * _e_fuzzy) {
        fuzzyRegion.insert(f);
    }
}

void MeshOperations::getPairwiseFuzzyRegion(const std::unordered_set<int> &patch_one,
                                            const std::unordered_set<int> &patch_two,
                                            std::unordered_set<int> &fuzzyRegion) {
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

void MeshOperations::generateFuzzyRegions(std::vector<std::unordered_set<int>> &printable_components,
                                          std::vector<Eigen::Vector3f> &printing_directions,
                                          std::vector<FuzzyNode*> &nodes) {
    // generate pairwise fuzzy regions
    // TODO: Can leverage multithreading here to make this faster
    for (int i = 0; i < printable_components.size(); i++) {
        for (int j = i+1; j < printable_components.size(); j++) {
            // pointer to fuzzyRegion to avoid the struct having to copy everything over on initialization
            // This will need to be cleaned up later
            unordered_set<int>* fuzzyRegion = new unordered_set<int>();
            getPairwiseFuzzyRegion(printable_components[i], printable_components[j], *fuzzyRegion);
            if (fuzzyRegion->size() == 0) {
                // If the pairwise patches don't share boundaries
                delete fuzzyRegion;
            } else {
                unordered_set<FuzzyNode*> neighbors;
                vector<int> patchDirections = {i, j};
                // This will need to be cleaned up later
                FuzzyNode* fuzzyNode = new FuzzyNode{fuzzyRegion, neighbors, patchDirections};
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
    // TODO: Can leverage multithreading to make this faster
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

void fuzzyDFS(unordered_set<FuzzyNode*> &visited,
              unordered_set<int> &regionFaces,
              unordered_set<int> &regionDirections,
              FuzzyNode* node) {
    if (visited.contains(node)) {
        return;
    }
    // Add the node's faces to the overall fuzzy region
    for (int i: *node->fuzzyRegion) {
        regionFaces.insert(i);
    }
    // Add the node's directions to the overall fuzzy region
    for (int d: node->patchDirections) {
        regionDirections.insert(d);
    }
    // Mark node as visited
    visited.insert(node);
    // Visit node neighbors
    for (FuzzyNode* n: node->neighbors) {
        fuzzyDFS(visited, regionFaces, regionDirections, n);
    }
}

// Combines the fuzzy regions
void MeshOperations::combineFuzzyRegions(std::vector<FuzzyNode*> &nodes,
                                         std::vector<std::unordered_set<int>> &fuzzyRegions,
                                         std::vector<std::unordered_set<int>> &fuzzyRegionDirections) {
    unordered_set<FuzzyNode*> visited;
    for (FuzzyNode* node: nodes) {
        if (visited.contains(node)) {
            continue;
        }
        unordered_set<int> fuzzyRegion;
        unordered_set<int> fuzzyDirections;
        fuzzyDFS(visited, fuzzyRegion, fuzzyDirections, node);
        fuzzyRegions.push_back(fuzzyRegion);
        fuzzyRegionDirections.push_back(fuzzyDirections);
    }
}

void MeshOperations::generateRefinedSegmentation(std::vector<std::unordered_set<int>> &printable_components,
                                                 std::vector<Eigen::Vector3f> &printing_directions,
                                                 std::vector<std::unordered_set<int>> &fuzzyRegions) {
    // Create pairwise fuzzy regions (FuzzyNodes)
    vector<FuzzyNode*> nodes;
    vector<unordered_set<int>> fuzzyRegionDirections;
    std::cout << "Making initial fuzzy regions..." << std::endl;
    generateFuzzyRegions(printable_components, printing_directions, nodes);
    std::cout << "Number of initial fuzzy regions: " << nodes.size() << std::endl;

    // Combine the fuzzy regions into the largest possible connected components
    makeFuzzyGraph(nodes);
    combineFuzzyRegions(nodes, fuzzyRegions, fuzzyRegionDirections);
    std::cout << "Number of total fuzzy regions: " << fuzzyRegions.size() << std::endl;
    std::cout << "Number of total fuzzy directions: " << fuzzyRegionDirections.size() << std::endl;
    std::cout << "Sample directions: " << std::endl;
    for (int i: fuzzyRegionDirections[0]) {
        std::cout << i << std::endl;
    }
    // After this point, we no longer need the nodes (these are heap allocated, so they can be freed)
    for (int node = 0; node < nodes.size(); node++) {
        delete nodes[node]->fuzzyRegion;
        delete nodes[node];
    }
    nodes.clear();

    // Visualize the fuzzy regions here before we cut them
    if (!_refined_skip_visualization) {
        visualize(fuzzyRegions);
    }

    // Begin optimizing the fuzzy regions
    // TODO: Can *maybe* use multithreading to parallelize this
    for (int region = 0; region < fuzzyRegions.size(); region++) {
        // Generate coefficients for this region
        std::unordered_map<std::pair<int, int>, double, PairHash> adjacent_face_coefficients;
        initializeFuzzyRegionCoefficients(fuzzyRegions[region], adjacent_face_coefficients);
        std::cout << "Created fuzzy region coefficients for " << adjacent_face_coefficients.size() << " adjacent faces" << std::endl;

        // Solve and update the printable components
        solveFuzzyRegion(printable_components, fuzzyRegions[region], fuzzyRegionDirections[region], adjacent_face_coefficients);
    }
}

// Compute refined smoothing coefficient between two faces
double MeshOperations::computeRefinedCoefficient(const int &face_one, const int &face_two) {
    // If faces are not connected, it is 0
    if (_adjacency[face_one][face_two] == false) {
        return 0.0;
    }

    // Otherwise find the overlapping edge (this code is lmao)
    // TODO: Turn this into a helper for the mesh class or something (when it gets refactored)
    Eigen::Vector3i face_one_verts = _F.row(face_one);
    Eigen::Vector3i face_two_verts = _F.row(face_two);

    bool vert_zero_shared = (face_one_verts(0) == face_two_verts(0)) || (face_one_verts(0) == face_two_verts(1)) || (face_one_verts(0) == face_two_verts(2));
    bool vert_one_shared = (face_one_verts(1) == face_two_verts(0)) || (face_one_verts(1) == face_two_verts(1)) || (face_one_verts(1) == face_two_verts(2));
    bool vert_two_shared = (face_one_verts(2) == face_two_verts(0)) || (face_one_verts(2) == face_two_verts(1)) || (face_one_verts(2) == face_two_verts(2));

    // Determine shared edge
    int vert_one = -1;
    int vert_two = -1;
    if (vert_zero_shared && vert_one_shared) {
        vert_one = face_one_verts(0);
        vert_two = face_one_verts(1);
    } else if (vert_one_shared && vert_two_shared) {
        vert_one = face_one_verts(1);
        vert_two = face_one_verts(2);
    } else if (vert_zero_shared && vert_two_shared) {
        vert_one = face_one_verts(0);
        vert_two = face_one_verts(2);
    }
    // Here, the helper function would return the vertices making up the edge

    // Compute the coefficient
    double edge_length = (_V.row(vert_one) - _V.row(vert_two)).norm();
    double edge_AO = getEdgeAO(std::make_pair(std::min(vert_one, vert_two), std::max(vert_one, vert_two)));

    return edge_length * (std::exp(_ambient_occlusion_lambda * edge_AO) - 1);
}

// Initialize coefficients to pass into the solver for adjacent faces
void MeshOperations::initializeFuzzyRegionCoefficients(const std::unordered_set<int> &fuzzy_region, std::unordered_map<std::pair<int, int>, double, PairHash> &adjacent_face_coefficients) {
    // Initialize map
    adjacent_face_coefficients.clear();

    // We will optimize over all faces in the fuzzy region and their neighbors
    for (const int &face : fuzzy_region) {
        // Neighbors
        for (Face* const &neighbor : _mesh.getFace(face)->neighbors) {
            std::pair<int, int> adjacent_faces = std::make_pair(std::min(face, neighbor->index), std::max(face, neighbor->index));
            // Skip repeated computation
            if (!adjacent_face_coefficients.contains(adjacent_faces)) {
                adjacent_face_coefficients[adjacent_faces] = computeRefinedCoefficient(adjacent_faces.first, adjacent_faces.second);
            }
        }
    }
}

// Function to actually interface with the solver
void MeshOperations::solveFuzzyRegion(std::vector<std::unordered_set<int>> &printable_components,
                                      const std::unordered_set<int> &fuzzy_region,
                                      const unordered_set<int> &fuzzy_region_directions,
                                      const std::unordered_map<std::pair<int, int>, double, PairHash> &adjacent_face_coefficients) {
    // Reset solver in case this is called multiple times
    clearSolver();

    // Variables
    std::vector<std::vector<const MPVariable*>> variables;
    // Maps face index to variables
    std::unordered_map<int, int> face_to_variable;
    // Maps printing direction to variables
    std::unordered_map<int, int> variable_to_direction;
    int num_region_directions = 0;

    // Populate direction mapping
    for (const int &direction : fuzzy_region_directions) {
        variable_to_direction[num_region_directions] = direction;
        num_region_directions++;
    }

    // Make variables
    std::cout << "Adding variables for " << fuzzy_region.size() << " faces in fuzzy region" << std::endl;
    for (const auto &[adjacent_faces, smoothing_cost] : adjacent_face_coefficients) {
        // Create direction variables for each face if they don't exist
        if (!face_to_variable.contains(adjacent_faces.first)) {
            addRefinedFaceVariable(adjacent_faces.first, fuzzy_region, printable_components, variable_to_direction, face_to_variable, variables);
        }
        if (!face_to_variable.contains(adjacent_faces.second)) {
            addRefinedFaceVariable(adjacent_faces.second, fuzzy_region, printable_components, variable_to_direction, face_to_variable, variables);
        }

        // Encode XOR variable for this pair
        for (int direction = 0; direction < num_region_directions; direction++) {
            addXORVariable(variables[face_to_variable.at(adjacent_faces.first)][direction], variables[face_to_variable.at(adjacent_faces.second)][direction], smoothing_cost);
        }
    }

    std::cout << "Added " << _solver->NumVariables() << " variables for " << face_to_variable.size() << " faces and " << num_region_directions << " directions" << std::endl;
    std::cout << "Solver has " << _solver->NumConstraints() << " constraints for " << adjacent_face_coefficients.size() << " pairs of adjacent faces" << std::endl;

    // Solve!
    std::cout << "Solving system. This may take a while..." << std::endl;
    MPObjective* const objective = _solver->MutableObjective();
    objective->SetMinimization();
    _solver->Solve();

    // Unpack the variables and update how the printable components changed
    for (const auto &[face, face_variable] : face_to_variable) {
        updatePrintableComponents(face, face_variable, fuzzy_region, printable_components, variable_to_direction, variables);
    }
}

// Add a new variable for a face
void MeshOperations::addRefinedFaceVariable(const int &face,
                                            const std::unordered_set<int> &fuzzy_region,
                                            const std::vector<std::unordered_set<int>> &printable_components,
                                            std::unordered_map<int, int> &variable_to_direction,
                                            std::unordered_map<int, int> &face_to_variable,
                                            std::vector<std::vector<const MPVariable*>> &variables) {
    // Create a vector of variables (for each direction) at the end
    int num_face_variables = variables.size();
    int num_region_directions = variable_to_direction.size();
    variables.emplace_back();
    variables[num_face_variables].resize(num_region_directions);

    // Add this variable so we can look it up
    face_to_variable[face] = num_face_variables;

    // Determine if this face should stay fixed
    bool in_fuzzy_region = fuzzy_region.contains(face);
    int original_component = -1;
    if (!in_fuzzy_region) {
        for (int component = 0; component < printable_components.size(); component++) {
            if (printable_components[component].contains(face)) {
                original_component = component;
                break;
            }
        }
    }

    // Populate the new entry with appropriate values
    for (int direction = 0; direction < num_region_directions; direction++) {
        if (in_fuzzy_region) {
            // Coefficient is 0 because this doesn't matter for the objective
            variables[num_face_variables][direction] = addVariable(0.0, 0.0, 1.0);
        } else {
            // Faces on boundary of fuzzy region must have constrained values
            if (variable_to_direction.at(direction) == original_component) {
                // In original region, must be printed in this direction
                variables[num_face_variables][direction] = addVariable(0.0, 1.0, 1.0);
            } else {
                // Not in original region, cannot be printed in this direction
                variables[num_face_variables][direction] = addVariable(0.0, 0.0, 0.0);
            }
        }
    }

    // Add constraint to make sure this face will have an assigned printing direction
    std::vector<double> coefficients;
    coefficients.resize(num_region_directions, 1.0);
    addConstraint(variables[num_face_variables], coefficients, 1.0, 1.0);
}

// Adjust printable components based on the value from a face
void MeshOperations::updatePrintableComponents(const int &face,
                                               const int &face_variable,
                                               const std::unordered_set<int> &fuzzy_region,
                                               std::vector<std::unordered_set<int>> &printable_components,
                                               std::unordered_map<int, int> &variable_to_direction,
                                               std::vector<std::vector<const MPVariable*>> &variables) {
    // For faces not in the fuzzy region, we don't want to update them
    if (!fuzzy_region.contains(face)) {
        std::cout << "Skipping update of face " << face << " since it is not in fuzzy region" << std::endl;
        return;
    }

    int num_printable_directions = variable_to_direction.size();
    int original_dir = -1;
    for (int component = 0; component < printable_components.size(); component++) {
        if (printable_components[component].contains(face)) {
            original_dir = component;
            printable_components[component].erase(face);
        }
    }

    int new_dir = -1;

    for (int direction_variable = 0; direction_variable < num_printable_directions; direction_variable++) {
        // Recover corresponding printing direction
        int direction = variable_to_direction.at(direction_variable);
        // If the solution is 0, the face is not printed in this direction
        if (variables[face_variable][direction_variable]->solution_value() == 0) {
            // Therefore, it should be removed from the corresponding set
            // This is done previously because of debug statements
        }

        // If the solution is 1, the face is printed in this direction
        if (variables[face_variable][direction_variable]->solution_value() == 1) {
            // Therefore it should be added to this printable component
            printable_components[direction].insert(face);
            new_dir = direction;
        }
    }

    if (new_dir != original_dir) {
        std::cout << "Moved face " << face << " from printable component " << original_dir << " to printable component " << new_dir << std::endl;
    }
}
