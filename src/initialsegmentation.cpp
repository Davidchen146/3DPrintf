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
    std::cout << "Populating matrix with cost of supports" << std::endl;
    populateSupportMatrix(patches, directions);
    std::cout << "Populating matrix with smoothing cost between pairs" << std::endl;
    populateSmoothingMatrix(patches);

    // OK WE ARE GOING TO DO THE THING
    _solver->Clear();

    // Allocate space for all the items poggers
    std::vector<std::vector<const MPVariable*>> printing_direction_vars;
    printing_direction_vars.resize(patches.size());
    for (int patch = 0; patch < patches.size(); patch++) {
        printing_direction_vars[patch].resize(_num_random_dir_samples);
    }

    // Support costs
    addSupportCosts(printing_direction_vars, patches);
    // Smoothing costs
    addSmoothingCosts(printing_direction_vars);

    // SOLVE THIS
    std::cout << "Solving initial segmentation optimization. This may take a while..." << std::endl;
    MPObjective* const objective = _solver->MutableObjective();
    objective->SetMinimization();
    _solver->Solve();
  
    // Use the solutions to generate the printable componenets
    std::cout << "Generating printable components" << std::endl;
    generatePrintableComponents(patches, printable_components, printing_direction_vars, directions, printing_directions);
}

// Subroutines used for Phase 2 (Initial Segmentation)
// Sample random directions
void MeshOperations::sampleRandomDirections(std::vector<Eigen::Vector3f> &directions) {
    directions.clear();

    // Generate axis-aligned directions only?
    if (_axis_only) {
        directions.emplace_back(1, 0, 0);
        directions.emplace_back(-1, 0, 0);
        directions.emplace_back(0, 1, 0);
        directions.emplace_back(0, -1, 0);
        directions.emplace_back(0, 0, 1);
        directions.emplace_back(0, 0, -1);
    } else {
        for (int i = 0; i < _num_random_dir_samples; i++) {
            Vector3f direction = generateRandomVector();

            // NOTE: assumes that directions starts off as an empty vector
            directions.push_back(direction);
        }
    }

    _num_random_dir_samples = directions.size();
}

// Determine if a face is overhanging and requires support
bool MeshOperations::isFaceOverhanging(const int face, const Eigen::Vector3f &direction) {
    Eigen::Vector3f faceNormal = getFaceNormal(face);

    // No supports on inside of shell (normals should be pointing towards printing direction)
    double dot = faceNormal.dot(direction);
    if (dot > 0) {
        return false;
    }

    double angle = acos(-dot);
    return angle < (std::numbers::pi / 2) - _printer_tolerance_angle;
}

// Determine if an edge is overhanging and requires support
bool MeshOperations::isEdgeOverhanging(const std::pair<int, int> &edge, const Eigen::Vector3f &direction) {
    Eigen::Vector3f edgeNormal = getEdgeNormal(edge);

    // No supports on inside of shell (normals should be pointing towards printing direction)
    double dot = edgeNormal.dot(direction);
    if (dot > 0) {
        return false;
    }

    double angle = acos(-dot);
    return angle < (std::numbers::pi / 2) - _printer_tolerance_angle;
}

// Determine if a vertex is overhanging and requires support
bool MeshOperations::isVertexOverhanging(const int vertex, const Eigen::Vector3f &direction) {
    Eigen::Vector3f vertexNormal = getVertexNormal(vertex);

    // No supports on inside of shell (normals should be pointing towards printing direction)
    double dot = vertexNormal.dot(direction);
    if (dot > 0) {
        return false;
    }

    double angle = acos(-dot);
    return angle < (std::numbers::pi / 2) - _printer_tolerance_angle;
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
            // The ray's direction should be opposite the normal and a different face
            if (intersected_face == face || getFaceNormal(intersected_face).dot(direction) < 0) {
                continue;
            }
            footing_faces.push_back(intersected_face);
        }
    }
}

// Compute support coefficient for a face in direction
double MeshOperations::computeSupportCoefficient(const int &face) {
    // Any face specified in the zero cost step does not incur a support cost
    if (_use_zero_cost_faces && _zero_cost_faces.contains(face)) {
        return 0;
    }

    // Area * ambient occlusion (exponentiated)
    // get area
    double area = getArea(face);
    double faceAO = getFaceAO(face);
    return area * std::pow(faceAO, _ambient_occlusion_supports_alpha);
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

// Assign results of the ILP to something we can return out
// Assumes the solver already has results assigned to variables
void MeshOperations::generatePrintableComponents(const std::vector<std::unordered_set<int>> &patches,
                                                 std::vector<unordered_set<int>> &printable_components,
                                                 const std::vector<std::vector<const MPVariable*>> &solutions,
                                                 const std::vector<Eigen::Vector3f> &patch_printing_directions,
                                                 std::vector<Eigen::Vector3f> &component_printing_directions) {
    // For each patch, determine its printing direction
    std::vector<int> patch_directions;
    patch_directions.resize(patches.size(), -1);
    std::unordered_set<int> used_printing_directions;

    for (int patch = 0; patch < patches.size(); patch++) {
        // Find the value
        for (int direction = 0; direction < _num_random_dir_samples; direction++) {
            std::cout << "Accessing patch: " << patch << " direction: " << direction << std::endl;
            std::cout << "Size of solutions " << solutions.size() << " element: " << solutions[patch].size() << std::endl;
            int solver_value = solutions[patch][direction]->solution_value();
            std::cout << "Accessed solver value: " << solver_value << std::endl;
            if (solver_value == 1) {
                // This is the optimal direction for this patch
                patch_directions[patch] = direction;
                used_printing_directions.insert(direction);
                break;
            }
        }
        assert(patch_directions[patch] != -1);
    }

    std::cout << "Assigning elements" << std::endl;

    // Group the patches based on their direction
    printable_components.resize(used_printing_directions.size());

    // Determine which printing directions were actually used and collect them
    std::unordered_map<int, int> direction_to_idx;
    for (const int &direction : used_printing_directions) {
        direction_to_idx[direction] = component_printing_directions.size();
        component_printing_directions.push_back(patch_printing_directions[direction]);
    }

    // Now we can group all the patches by the printing directions
    for (int patch = 0; patch < patches.size(); patch++) {
        for (const int &face : patches[patch]) {
            printable_components[direction_to_idx[patch_directions[patch]]].insert(face);
        }
    }
    std::cout << "Printable component assignment completed" << std::endl;

    // How many faces got assigned to each direction
    // DEBUG Information
    for (int component = 0; component < printable_components.size(); component++) {
        Eigen::Vector3f direction = component_printing_directions[component];
        float x = direction(0);
        float y = direction(1);
        float z = direction(2);
        std::cout << "Direction (" << x << ", " << y << ", " << z << ") has " << printable_components[component].size() << " faces" << std::endl;
    }
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
        // Determine what faces need support in this direction
        std::unordered_set<int> supporting_faces;
        Eigen::Vector3f dir = directions[dirInd];

        for (int face = 0; face < _F.rows(); face++) {
            // Todo: determine whether face requires support and needs to incur a cost
            bool costNeeded = false;
            // case a: face normal has angle w/ base greater than a threshold (paper uses 55)
            if (isFaceOverhanging(face, dir)) {
                costNeeded = true;
            }
            else {
                // get the edges to check each of them for overhang
                int v1 = _F.row(face)(0);
                int v2 = _F.row(face)(1);
                int v3 = _F.row(face)(2);
                std::pair<int, int> e1 = (v1 < v2) ? std::make_pair(v1, v2) : std::make_pair(v2, v1);
                std::pair<int, int> e2 = (v2 < v3) ? std::make_pair(v2, v3) : std::make_pair(v3, v2);
                std::pair<int, int> e3 = (v3 < v1) ? std::make_pair(v3, v1) : std::make_pair(v1, v3);
                // case b, c: edge or vertex normals form  angle w/ base greater than a threshold (paper uses 55)
                if (isVertexOverhanging(v1, dir) || isVertexOverhanging(v2, dir) || isVertexOverhanging(v3, dir)) {
                    costNeeded = true;
                }
                else if (isEdgeOverhanging(e1, dir) || isEdgeOverhanging(e2, dir) || isEdgeOverhanging(e3, dir)) {
                    costNeeded = true;
                }
            }
            // This face is supported, determine any footing faces for this face
            if (costNeeded) {
                supporting_faces.insert(face);
                std::vector<int> newFootedFaces;
                findFootingFaces(face, dir, newFootedFaces);
                for (const int &newFace : newFootedFaces) {
                    supporting_faces.insert(newFace);
                }
            }
        }

        // We have determined all faces that need support (either overhanging or footing), compute the cost for each patch
        for (int patchInd = 0; patchInd < patches.size(); patchInd++) {
            double patch_cost = 0.0;
            // Patch cost is entirely determined by supporting faces
            for (int f : patches[patchInd]) {
                if (supporting_faces.contains(f)) {
                    patch_cost += computeSupportCoefficient(f);
                    assert(patch_cost >= 0.0);
                }
            }

            // Update value
            _supportCoefficients(patchInd, dirInd) = patch_cost;
        }
    }
}

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

void MeshOperations::addSupportCosts(std::vector<std::vector<const MPVariable*>> &variables, const std::vector<std::unordered_set<int>> &patches) {
    // Initialize variables
    int numPatches = patches.size();
    for (int patch = 0; patch < numPatches; patch++) {
        // Create a variable for this patch indicating if it's printed in this direction
        std::vector<const MPVariable*> patch_direction_vars;

        for (int printing_direction = 0; printing_direction < _num_random_dir_samples; printing_direction++) {
            const MPVariable* new_dir_var = addVariable(_supportCoefficients(patch, printing_direction), 0.0, 1.0);
            variables[patch][printing_direction] = new_dir_var;
            patch_direction_vars.push_back(new_dir_var);
        }

        // Encode constraint that this patch can only have one printed direction
        std::vector<double> patch_constraint_coefficients;
        patch_constraint_coefficients.resize(patch_direction_vars.size(), 1.0);
        addConstraint(patch_direction_vars, patch_constraint_coefficients, 1.0, 1.0);
    }
}

void MeshOperations::addSmoothingCosts(std::vector<std::vector<const MPVariable*>> &variables) {
    LOG(INFO) << "Num Neighboring Patch Pairs: " << _smoothingCoefficients.size();
    LOG(INFO) << "Num Variables Pre-XOR: " << _solver->NumVariables();
    LOG(INFO) << "Num Constraints Pre-XOR: " << _solver->NumConstraints();

    // For each pair of adjacent faces...
    for (const auto& [patch_pair, smoothing_cost] : _smoothingCoefficients) {
        // For each Direction:
        for (int direction = 0; direction < _num_random_dir_samples; direction++) {
            // Create a new variable corresponding to the relevant XOR in the solver
            const MPVariable* patch_1_dir_indicator = variables[patch_pair.first][direction];
            const MPVariable* patch_2_dir_indicator = variables[patch_pair.second][direction];

            addXORVariable(patch_1_dir_indicator, patch_2_dir_indicator, smoothing_cost);
        }
    }

    LOG(INFO) << "Num Variables Post-XOR: " << _solver->NumVariables();
    LOG(INFO) << "Num Constraints Post-XOR: " << _solver->NumConstraints();
}
