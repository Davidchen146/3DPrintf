#include "src/meshoperations.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/rotation_matrix_from_directions.h>

// Linear interpolation function
Eigen::Vector3d MeshOperations::lerp(const Eigen::Vector3d& color1, const Eigen::Vector3d& color2, double t) {
    return color1 + t * (color2 - color1);
}

// Blue is small, green/yellow is medium, and red is high
Eigen::Vector3d MeshOperations::mapValueToColor(double value, double max_value) {
    // Interpolate between blue (low AO), green/yellow (medium AO), and red (high AO)
    if (value <= (max_value / 2)) {
        // Blue to green/yellow interpolation
        return lerp(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 1.0, 0.0), value * 2 / max_value);
    } else {
        // Green/yellow to red interpolation
        return lerp(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), (value - (max_value / 2)) / (max_value / 2));
    }
}

// Visualize groups with random colors
void MeshOperations::visualize(const vector<unordered_set<int>>& coloringGroups) {
    // generate a certain number of colors based on coloringGroups
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);
    Eigen::Vector3d gray(65.0f / 255, 65.0f / 255, 65.0f / 255);
    for (int i = 0; i < _faces.size(); i++) {
        C.row(i) = gray;
    }

    faceToGroup.clear();
    for (int i = 0; i < coloringGroups.size(); i++) {
        std::unordered_set<int> group_i = coloringGroups[i];
        for (auto j = group_i.begin(); j != group_i.end(); j++) {
            int face_index = *j;
            assert(!faceToGroup.contains(face_index));
            faceToGroup[face_index] = i;
        }
    }

    groupToColor.clear();
    for (int i = 0; i < coloringGroups.size(); i++) {
        double m_red = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double m_green = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double m_blue = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        Vector3d RGB = {m_red, m_green, m_blue};
        groupToColor[i] = RGB;
    }

    for (int i = 0; i < _faces.size(); i++) {
        if (faceToGroup.contains(i)) {
            C.row(i) = groupToColor[faceToGroup[i]];
        } else {
            C.row(i) = gray;
        }
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}

// Visualizes supported faces as red and unsupported faces as green
void MeshOperations::visualizePrintableComponents(const std::vector<std::unordered_set<int>> &printable_components, const std::vector<Eigen::Vector3f> &printing_directions) {
    // Only visualize individual printable components and orient the viewer so that these directions point upwards
    int num_components = printable_components.size();
    assert(printing_directions.size() == printable_components.size());

    for (int component = 0; component < num_components; component++) {
        // Rotate all the vertices so the printing direction points upward
        Eigen::MatrixXf component_vertices;
        component_vertices.resize(_V.rows(), 3);
        Eigen::Vector3f printing_direction = printing_directions[component];
        // This matrix will rotate the printing direction to directly upwards, so it will rotate the mesh so the printing direction is upwards
        Eigen::Matrix3f rotation = igl::rotation_matrix_from_directions(printing_direction, Eigen::Vector3f(0.0, 1.0, 0.0)).transpose();
        component_vertices = _V * rotation;

        // Determine what faces we want to visualize
        int faces_to_visualize = printable_components[component].size();
        Eigen::MatrixXi component_faces;
        component_faces.resize(faces_to_visualize, 3);

        // Add color corresponding to the support value computed for the faces
        std::unordered_set<int> supported_faces;
        for (const int &face : printable_components[component]) {
            // Determine if face is supported
            bool is_face_supported = false;
            is_face_supported |= isFaceOverhanging(face, printing_direction);
            // get the edges to check each of them for overhang
            int v1 = _F.row(face)(0);
            int v2 = _F.row(face)(1);
            int v3 = _F.row(face)(2);
            std::pair<int, int> e1 = (v1 < v2) ? std::make_pair(v1, v2) : std::make_pair(v2, v1);
            std::pair<int, int> e2 = (v2 < v3) ? std::make_pair(v2, v3) : std::make_pair(v3, v2);
            std::pair<int, int> e3 = (v3 < v1) ? std::make_pair(v3, v1) : std::make_pair(v1, v3);
            // case b, c: edge or vertex normals form  angle w/ base greater than a threshold (paper uses 55)
            is_face_supported |= isVertexOverhanging(v1, printing_direction) || isVertexOverhanging(v2, printing_direction) || isVertexOverhanging(v3, printing_direction);
            is_face_supported |= isEdgeOverhanging(e1, printing_direction) || isEdgeOverhanging(e2, printing_direction) || isEdgeOverhanging(e3, printing_direction);

            // BUT IS THE FACE SUPPORTED?
            if (is_face_supported) {
                supported_faces.insert(face);
                // Determine footing faces
                std::vector<int> newFootedFaces;
                findFootingFaces(face, printing_direction, newFootedFaces);
                for (const int &newFace : newFootedFaces) {
                    supported_faces.insert(newFace);
                }
            }
        }

        Eigen::MatrixXd color;
        color.resize(faces_to_visualize, 3);
        int current_face = 0;
        for (const auto &face : printable_components[component]) {
            // This is a face we want to visualize
            component_faces.row(current_face) = _F.row(face);
            if (supported_faces.contains(face)) {
                if (_zero_cost_faces.contains(face)) {
                    // Blue faces are supported but have cost ignored
                    color.row(current_face) = Eigen::Vector3d(0, 0, 1);
                } else {
                    // Red faces incur cost for supports
                    color.row(current_face) = Eigen::Vector3d(1, 0, 0);
                }
            } else {
                color.row(current_face) = Eigen::Vector3d(0, 1, 0);
            }
            current_face++;
        }

        // Visualize the faces
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(component_vertices.cast<double>(), component_faces);
        viewer.data().set_colors(color);
        viewer.launch();
    }
}

// Debug to visualize outputs of the angular distance metric
void MeshOperations::visualizeAngularDistance() {
    Eigen::MatrixXd color;
    color.resize(_F.rows(), 3);

    // For each face, assign it the average angular distance of its neighbors
    std::unordered_map<int, double> face_to_dist;

    double max_dist = 0;
    for (int face = 0; face < _F.rows(); face++) {
        double avg_angular_dist = 0.0;
        const Face *current_face = _mesh.getFace(face);
        for (const Face* face : current_face->neighbors) {
            avg_angular_dist += getAngularDistance(current_face->index, face->index);
        }
        avg_angular_dist /= current_face->neighbors.size();
        max_dist = max(max_dist, avg_angular_dist);
        face_to_dist[face] = avg_angular_dist;
    }

    // VISUALIZE THAT THING
    for (int face = 0; face < _F.rows(); face++) {
        color.row(face) = mapValueToColor(face_to_dist[face], max_dist);
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(color);
    viewer.launch();
}

// Debug to visualize outputs of the weighted distance
void MeshOperations::visualizeWeightedDistance() {
    Eigen::MatrixXd color;
    color.resize(_F.rows(), 3);

    // For each face, assign it the average angular distance of its neighbors
    std::unordered_map<int, double> face_to_dist;

    double max_dist = 0;
    for (int face = 0; face < _F.rows(); face++) {
        double avg_angular_dist = 0.0;
        const Face *current_face = _mesh.getFace(face);
        for (const Face* face : current_face->neighbors) {
            avg_angular_dist += getWeightedDistance(current_face->index, face->index);
        }
        avg_angular_dist /= current_face->neighbors.size();
        max_dist = max(max_dist, avg_angular_dist);
        face_to_dist[face] = avg_angular_dist;
    }

    // VISUALIZE THAT THING
    for (int face = 0; face < _F.rows(); face++) {
        color.row(face) = mapValueToColor(face_to_dist[face], max_dist);
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(color);
    viewer.launch();
}

// Debug to visualize support costs given a direction
// Does not require an oversegmentation (all values are computed per-face)
void MeshOperations::visualizeSupportCosts(const Eigen::Vector3f &printing_direction) {
    Eigen::MatrixXd color;
    color.resize(_F.rows(), 3);

    // For each face, assign it the support cost for being printed in this direction
    // Determine supporting faces
    std::unordered_set<int> supported_faces;
    for (int face = 0; face < _F.rows(); face++) {
        // Is face supported?
        bool is_face_supported = false;
        is_face_supported |= isFaceOverhanging(face, printing_direction);
        // get the edges to check each of them for overhang
        int v1 = _F.row(face)(0);
        int v2 = _F.row(face)(1);
        int v3 = _F.row(face)(2);
        std::pair<int, int> e1 = (v1 < v2) ? std::make_pair(v1, v2) : std::make_pair(v2, v1);
        std::pair<int, int> e2 = (v2 < v3) ? std::make_pair(v2, v3) : std::make_pair(v3, v2);
        std::pair<int, int> e3 = (v3 < v1) ? std::make_pair(v3, v1) : std::make_pair(v1, v3);
        // case b, c: edge or vertex normals form  angle w/ base greater than a threshold (paper uses 55)
        is_face_supported |= isVertexOverhanging(v1, printing_direction) || isVertexOverhanging(v2, printing_direction) || isVertexOverhanging(v3, printing_direction);
        is_face_supported |= isEdgeOverhanging(e1, printing_direction) || isEdgeOverhanging(e2, printing_direction) || isEdgeOverhanging(e3, printing_direction);

        // BUT IS THE FACE SUPPORTED?
        if (is_face_supported) {
            supported_faces.insert(face);
            // Determine footing faces
            std::vector<int> newFootedFaces;
            findFootingFaces(face, printing_direction, newFootedFaces);
            for (const int &newFace : newFootedFaces) {
                supported_faces.insert(newFace);
            }
        }
    }

    // Determine max cost
    double max_support_cost = 0;
    for (const int &face : supported_faces) {
        double support_cost = computeSupportCoefficient(face);
        if (support_cost > max_support_cost) {
            max_support_cost = support_cost;
        }
    }

    // Produce appropriate outputs
    for (int face = 0; face < _F.rows(); face++) {
        if (supported_faces.contains(face)) {
            color.row(face) = mapValueToColor(computeSupportCoefficient(face), max_support_cost);
            Eigen::Vector3f normal = getFaceNormal(face);
        } else {
            color.row(face) = mapValueToColor(0, max_support_cost);
        }
    }

    // Visualize!
    // Rotate!
    Eigen::MatrixXf rotated_vertices;
    rotated_vertices.resize(_V.rows(), 3);
    Eigen::Matrix3f rotation = igl::rotation_matrix_from_directions(printing_direction, Eigen::Vector3f(0.0, 1.0, 0.0)).transpose();
    std::cout << "What is the matrix: " << rotation << std::endl;
    rotated_vertices = _V * rotation;

    // Visualize the faces
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(rotated_vertices.cast<double>(), _F);
    viewer.data().set_colors(color);
    viewer.launch();
}

// visualize smoothing costs
// takes in the patches from the oversegmentation
void MeshOperations::visualizeSmoothingCosts(const std::vector<std::unordered_set<int>>& patches) {
    populateSmoothingMatrix(patches);
    std::unordered_map<int, double> patchToCost;
    std::unordered_map<int, int> adjacentPatchCount;
    for (const auto& pair : _smoothingCoefficients) {
        std::pair<int, int> patchPair = pair.first;
        double cost = pair.second;
        if (!patchToCost.contains(patchPair.first)) {
            patchToCost[patchPair.first] = cost;
            adjacentPatchCount[patchPair.first] = 1;
        } else {
            patchToCost[patchPair.first] += cost;
            adjacentPatchCount[patchPair.first] += 1;
        }
        if (!patchToCost.contains(patchPair.second)) {
            patchToCost[patchPair.second] = cost;
            adjacentPatchCount[patchPair.second] = 1;
        } else {
            patchToCost[patchPair.second] += cost;
            adjacentPatchCount[patchPair.second] += 1;
        }
    }

    double max_cost = 0;
    for (const auto& pair : patchToCost) {
        std::unordered_set<std::pair<int, int>, PairHash> edge_set;
        getPatchBoundary(patches[pair.first], edge_set);

        // Normalize by size of patch boundary
        double edge_length = 0;
        for (const auto &edge : edge_set) {
            edge_length += (_V.row(edge.first) - _V.row(edge.second)).norm();
        }

        patchToCost[pair.first] /= edge_length;
        max_cost = max(max_cost, patchToCost[pair.first]);
    }
    std::unordered_map<int, Vector3d> patchToColor;
    for (const auto& pair: patchToCost) {
        // get the color of the patch
        patchToColor[pair.first] = mapValueToColor(pair.second, max_cost);
    }
    std::unordered_map<int, int> faceToPatch;
    for (int i = 0; i < patches.size(); i++) {
        std::unordered_set<int> patchI = patches[i];
        for (int j : patchI) {
            faceToPatch[j] = i;
        }
    }
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);
    for (int i = 0; i < _faces.size(); i++) {
        C.row(i) = patchToColor[faceToPatch[i]];
    }
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}

// Debug to visualize outputs of the AO subroutine
// Note that small values (blue) are occluded, and large values (red) are visible
void MeshOperations::visualizeFaceAO() {
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);

    double max_ao = 0;
    vector<double> ao_vals;
    for (int i = 0; i < _faces.size(); i++) {
        double ao = getFaceAO(i);
        max_ao = max(ao, max_ao);
        ao_vals.push_back(ao);
    }

    for (int i = 0; i < _faces.size(); i++) {
        C.row(i) = mapValueToColor(ao_vals[i], max_ao);
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}

void MeshOperations::visualizeEdgeAO() {
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);

    double max_ao = 0;
    vector<double> ao_vals;
    for (int i = 0; i < _faces.size(); i++) {
        Eigen::Vector3i vertexIndices = _faces[i];
        std::pair<int, int> e1 = _mesh.getSortedPair(vertexIndices[0], vertexIndices[1]);
        std::pair<int, int> e2 = _mesh.getSortedPair(vertexIndices[1], vertexIndices[2]);
        std::pair<int, int> e3 = _mesh.getSortedPair(vertexIndices[0], vertexIndices[2]);
        double ao1 = getEdgeAO(e1);
        double ao2 = getEdgeAO(e2);
        double ao3 = getEdgeAO(e3);
        double ao = (ao1 + ao2 + ao3) / 3;
        max_ao = max(ao, max_ao);
        ao_vals.push_back(ao);
    }

    for (int i = 0; i < _faces.size(); i++) {
        C.row(i) = mapValueToColor(ao_vals[i], max_ao);
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}

void MeshOperations::visualizePrintableVolumes(const std::vector<std::unordered_set<int>> &printable_components,
                                               const std::vector<Eigen::Vector3f> &printing_directions,
                                               const std::vector<std::vector<Eigen::Vector4i>> &printable_volumes) {
    if (printable_components.size() != printable_volumes.size()) {
        std::cerr << "NUMBER OF PRINTABLE COMPONENTS SHOULD BE SAME AS NUMBER OF PRINTABLE VOLUMES" << std::endl;
        return;
    }
    if (groupToColor.size() == 0) {
        std::cerr << "SHOULD CALL visualize() before visualizePrintableVolumes()" << std::endl;
        return;
    }
    int num_components = printable_components.size();
    for (int component = 0; component < num_components; component++) {
        Vector3d RGB = groupToColor[component];

        // Rotate all the vertices so the printing direction points upward
        Eigen::MatrixXd component_vertices;
        component_vertices.resize(_V.rows(), 3);
        Eigen::Vector3d printing_direction = printing_directions[component].cast<double>();
        // this matrix will rotate the printing direction directly upwards, so it will rotate the mesh so the printing direction is upwards
        Eigen::Matrix3d rotation = igl::rotation_matrix_from_directions(printing_direction, Eigen::Vector3d(0.0, 1.0, 0.0)).transpose();
        component_vertices = _V.cast<double>() * rotation;

        // determine what faces we want to visualize
        int faces_to_visualize = printable_components[component].size();
        Eigen::MatrixXi component_faces;
        component_faces.resize(faces_to_visualize, 3);

        Eigen::MatrixXd color;
        color.resize(faces_to_visualize, 3);
        int current_face = 0;
        for (const auto &face : printable_components[component]) {
            // this is a face we want to visualize
            component_faces.row(current_face) = _F.row(face);
            color.row(current_face) = RGB;
            current_face++;
        }

        // visualize the printable component
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(component_vertices, component_faces);
        viewer.data().set_colors(color);
        viewer.launch();

        // visualize the printable volume
        std::vector<Eigen::Vector4i> volume = printable_volumes[component];
        std::vector<Eigen::Vector3i> surfaceFaces;
        extractSurface(volume, surfaceFaces);

        color.resize(surfaceFaces.size(), 3);
        Eigen::MatrixXi faces;
        faces.resize(surfaceFaces.size(), 3);
        for (int i = 0; i < surfaceFaces.size(); i++) {
            faces.row(i) = surfaceFaces[i];
            color.row(i) = RGB;
        }

        component_vertices.setZero();
        component_vertices.resize(_TV.rows(), 3);
        component_vertices = _TV * rotation;

        igl::opengl::glfw::Viewer anotherViewer;
        anotherViewer.data().set_mesh(component_vertices, faces);
        anotherViewer.data().set_colors(color);
        anotherViewer.launch();
    }
}

