#include "qmath.h"
#include "src/meshoperations.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <QString>
#include <igl/invert_diag.h>
#include <igl/adjacency_matrix.h>

#include <igl/per_vertex_attribute_smoothing.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/copyleft/cgal/string_to_mesh_boolean_type.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/rotation_matrix_from_directions.h>
#include <igl/remove_unreferenced.h>

#include <igl/opengl/glfw/Viewer.h>

// Mesh the interior of a surface mesh (V,F) using tetgen
// @param[in] V #V x 3 vertex position list
// @param[in] F #F x 3 indices into V
// @param[in] switches string of tetgen options, see https://wias-berlin.de/software/tetgen/switches.html
// @param[out] TV #TV x 3 vertex position list
// @param[out] TT #TT x 4 list of tet face indices
// @param[out] TF #TF x 3 list of triangle face indices
// (contents of TF are the same as _F --> IF "p" option is specified)
void MeshOperations::tetrahedralizeMesh() {
    // Auto generate volume if not specified in config file
    double volume = _t_volume;
    if (volume == 0) {
        // i just took the options from the example, not 100% what to use here
        VectorXd area;
        igl::doublearea(_V.cast<double>(),_F,area);
        double area_avg = area.mean();
        double edge_length = sqrt(4 * area_avg / sqrt(3));
        volume = pow(edge_length, 3) / (6 * sqrt(2));
        volume /= 2;
        std::cout << "Generating automatic volume constraint of: " << volume << std::endl;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5) << volume;
    std::string volume_string = oss.str();

    // Generate options using specified quality and volume parameters
    std::string options = "pq" + std::to_string(_t_quality) + "Ya" + volume_string + "n";
    std::cout << "Generating tetrahedralization with the following options: " << options << std::endl;
    igl::copyleft::tetgen::tetrahedralize(_V.cast<double>(), _F, options, _TV, _TT, _TF);
}

Eigen::Vector3d MeshOperations::computeTetCentroid(Eigen::Vector4i &tetrahedron) {
    Vector3d tetVertex1 = _TV.row(tetrahedron[0]);
    Vector3d tetVertex2 = _TV.row(tetrahedron[1]);
    Vector3d tetVertex3 = _TV.row(tetrahedron[2]);
    Vector3d tetVertex4 = _TV.row(tetrahedron[3]);
    return (tetVertex1 + tetVertex2 + tetVertex3 + tetVertex4) / 4.f;
}

void MeshOperations::getComponentBoundingBox(const std::unordered_set<int> &component, Eigen::Vector3d& min, Eigen::Vector3d& max) {
    std::unordered_set<int> vertexIndices;
    for (int f : component) {
        Vector3i face = _faces[f];
        if (!vertexIndices.contains(face[0])) {
            vertexIndices.insert(face[0]);
        }
        if (!vertexIndices.contains(face[1])) {
            vertexIndices.insert(face[1]);
        }
        if (!vertexIndices.contains(face[2])) {
            vertexIndices.insert(face[2]);
        }
    }
    MatrixXf V;
    V.resize(vertexIndices.size(), 3);
    int num = 0;
    for (int vertex : vertexIndices) {
        V.row(num) = _vertices[vertex];
        num++;
    }
    // get the bounding box of printable component?
    min = _V.cast<double>().colwise().minCoeff();
    max = _V.cast<double>().colwise().maxCoeff();
}


void MeshOperations::partitionVolume(const std::vector<std::unordered_set<int>> &printable_components,
                                     std::vector<std::vector<Eigen::Vector4i>> &printable_volumes) {
    printable_volumes.clear();
    for (int i = 0; i < printable_components.size(); i++) {
        printable_volumes.push_back({});
    }

    std::vector<Eigen::Vector3d> boxMin;
    std::vector<Eigen::Vector3d> boxMax;
    boxMin.resize(printable_components.size());
    boxMax.resize(printable_components.size());
    for (int i = 0; i < printable_components.size(); i++) {
        Eigen::Vector3d min;
        Eigen::Vector3d max;
        getComponentBoundingBox(printable_components[i], min, max);
        boxMin[i] = min;
        boxMax[i] = max;
    }

    // Determine printable component for this tetrahedra by casting random rays in a stratified manner
    int n = _num_random_rays;
    std::vector<Eigen::Vector3f> directions(n);
    float phi = M_PI * (sqrt(5) - 1);
    for (int i = 0; i < n; i++) {
        float y = 1 - ((float) i / (n - 1)) * 2;
        float radius = sqrt(1 - y * y);
        float theta = phi * i;
        float x = cos(theta) * radius;
        float z = sin(theta) * radius;
        directions[i] = Vector3f(x, y, z);
    }

    // This raycasting operation requires grouping faces
    if (faceToGroup.size() == 0) {
        // TODO: Can initialize this hashmap here, I think?
        // Shouldn't need a call to visualize and it can be bypassed
        std::cerr << "MUST CALL visualize() first to initialize faceToGroup" << std::endl;
        return;
    }

    for (int i = 0; i < _TT.rows(); i++) {
        Eigen::Vector4i tetrahedron = _TT.row(i).cast<int>();
        Eigen::Vector3f centroid = computeTetCentroid(tetrahedron).cast<float>();

        float minDistance = std::numeric_limits<float>::max();
        int boundaryFace = -1;
        for (Eigen::Vector3f& direction : directions) {
            float distance;
            int intersectedFace = getIntersectionWithDistance(centroid, direction, distance);
            if (intersectedFace >= 0 && distance < minDistance && distance > 0) {
                int groupNum = faceToGroup[intersectedFace];
                Eigen::Vector3d min = boxMin[groupNum];
                Eigen::Vector3d max = boxMax[groupNum];
                if (centroid[0] >= min[0] && centroid[0] <= max[0] && centroid[1] >= min[1] && centroid[1] <= max[1] && centroid[2] >= min[2] && centroid[2] <= max[2]) {
                    minDistance = distance;
                    boundaryFace = intersectedFace;
                }
            }
            if (intersectedFace == -1) {
                //std::cerr << "DID NOT HIT A FACE ON THE BOUNDARY?" << std::endl;
            }
        }
        int groupNum = faceToGroup[boundaryFace];
        printable_volumes[groupNum].push_back(tetrahedron);
    }

    // let's do manual laplacian smoothing

    // 1. iterate over the volumes
    for (int i = 0; i < printable_volumes.size(); i++) {
        std::vector<Eigen::Vector4i> volume = printable_volumes[i];

        // transfer contents of volume into a matrixXi
        Eigen::MatrixXi tets;
        tets.resize(volume.size(), 4);
        for (int j = 0; j < volume.size(); j++) {
            tets.row(j) = volume[j];
        }

        // 2. get the boundary faces using boundary facets
        Eigen::MatrixXi boundary_faces;
        igl::boundary_facets(tets, boundary_faces);

        //boundary faces should now have all faces that surround the mesh
        // identify the subset of faces that are new to this volume (i.e. not in the original mesh)
        std::unordered_set<int> newFaces; // we will use this to ID indices based on boundary_faces and it alone
        for (int j = 0; j < boundary_faces.rows(); j++) {
            Eigen::Vector3i face = boundary_faces.row(j);
            bool isNew = true;
            for (int k = 0; k < _F.rows(); k++) {
                Eigen::Vector3i originalFace = _F.row(k);
                if (face == originalFace) {
                    isNew = false;
                    break;
                }
            }
            if (isNew) {
                newFaces.insert(j);
            }
        }

        // using the new faces, populate a hashmap that maps a vertex included among the new faces to all of its neighbors
        std::unordered_map<int, std::unordered_set<int>> vertexToNeighbors;
        for (int j : newFaces) {
            Eigen::Vector3i face = boundary_faces.row(j);
            for (int k = 0; k < 3; k++) {
                int vertex = face[k];
                if (vertexToNeighbors.find(vertex) == vertexToNeighbors.end()) {
                    vertexToNeighbors[vertex] = {};
                }
                for (int l = 0; l < 3; l++) {
                    if (l != k) {
                        vertexToNeighbors[vertex].insert(face[l]);
                    }
                }
            }
        }

        // for each vertex in vertexToNeighbors determine the average of its neighbors' positions then reassign the corresponding vertex position with this new average scaled by alpha
        double scale = _smoothing_weight;
        // note: 20
        for (int iteration = 0; iteration < _smoothing_iterations; iteration++) {
            for (auto& [vertex, neighbors] : vertexToNeighbors) {
                Eigen::Vector3d average = Eigen::Vector3d::Zero();
                for (int neighbor : neighbors) {
                    average += _TV.row(neighbor).transpose();
                }
                if (!neighbors.empty()) { // avoid division by zero
                    average /= static_cast<double>(neighbors.size());
                }
                _TV.row(vertex) += scale * (average.transpose() - _TV.row(vertex));
            }
        }
    }

    // ALTERNATIVE APPROACH
    // need igl::adjacency_list which constructs the graph adjacency list of a given mesh (V, F)
    // gives you a vector<vector<int>> containing at row i the adjacent vertices of vertex i
    // but if i have the adjacent vertices
}


void MeshOperations::getFacesFromTet(const std::vector<Eigen::Vector4i>& volume, Eigen::MatrixXi& faces, std::unordered_set<Vector3i, Vector3iHash, Vector3iEqual>& exposedFaces) {
    std::unordered_map<Vector3i, int, Vector3iHash, Vector3iEqual> faceMap;
    // (theoretically Vector3iEqual will sort the vector) --> so duplicate faces with different orderings won't exist in the set
    for (int i = 0; i < volume.size(); i++) {
        Eigen::Vector4i tet = volume[i];
        Eigen::Vector3i face1 = {tet[0], tet[1], tet[2]};
        Eigen::Vector3i face2 = {tet[0], tet[1], tet[3]};
        Eigen::Vector3i face3 = {tet[0], tet[2], tet[3]};
        Eigen::Vector3i face4 = {tet[1], tet[2], tet[3]};
        if (faceMap.contains(face1)) {
            faceMap[face1] += 1;
        } else {
            faceMap[face1] = 1;
        }
        if (faceMap.contains(face2)) {
            faceMap[face2] += 1;
        } else {
            faceMap[face2] = 1;
        }
        if (faceMap.contains(face3)) {
            faceMap[face3] += 1;
        } else {
            faceMap[face3] = 1;
        }
        if (faceMap.contains(face4)) {
            faceMap[face4] += 1;
        } else {
            faceMap[face4] = 1;
        }
    }
    faces.resize(faceMap.size(), 3);
    int rowNum = 0;
    exposedFaces.clear();
    for (const std::pair<Vector3i, int>& pair : faceMap) {
        Vector3i face = pair.first;
        faces.row(rowNum) = face;
        int count = pair.second;
        if (count == 1) {
            exposedFaces.insert(face);
        }
        rowNum++;
    }
}

void MeshOperations::pruneVolume(std::vector<std::vector<Eigen::Vector4i>> &printable_volumes) {
    // each Eigen::Vector4i represents a tetrahedron --> each int is an index into TV
    for (int i = 0; i < printable_volumes.size(); i++) {
        MatrixXi faces;
        std::unordered_set<Vector3i, Vector3iHash, Vector3iEqual> exposedFaces;
        getFacesFromTet(printable_volumes[i], faces, exposedFaces);
        vector<vector<double>> A; // containing at row i the adjacent vertices of vertex i
        // from the printable volumes get the faces
        igl::adjacency_list(faces, A);
        std::vector<Eigen::Vector4i> volume = printable_volumes[i];
        // for (Eigen::Vector4i& tetrahedron : volume) {
        std::vector<int> toErase;
        for (int j = 0; j < volume.size(); j++) {
            Eigen::Vector4i tetrahedron = volume[j];
            bool v1_dangling = A[tetrahedron[0]].size() == 3;
            bool v2_dangling = A[tetrahedron[1]].size() == 3;
            bool v3_dangling = A[tetrahedron[2]].size() == 3;
            bool v4_dangling = A[tetrahedron[3]].size() == 3;
            int total_true = v1_dangling + v2_dangling + v3_dangling + v4_dangling;
            if (total_true >= 2) {
                // remove the tetrahedron from the printable volume
                toErase.push_back(j);
            } else {
                Eigen::Vector3i face1 = {tetrahedron[0], tetrahedron[1], tetrahedron[2]};
                Eigen::Vector3i face2 = {tetrahedron[0], tetrahedron[1], tetrahedron[3]};
                Eigen::Vector3i face3 = {tetrahedron[0], tetrahedron[2], tetrahedron[3]};
                Eigen::Vector3i face4 = {tetrahedron[1], tetrahedron[2], tetrahedron[3]};
                bool f1_exposed = exposedFaces.contains(face1);
                bool f2_exposed = exposedFaces.contains(face2);
                bool f3_exposed = exposedFaces.contains(face3);
                bool f4_exposed = exposedFaces.contains(face4);
                int num_exposed = f1_exposed + f2_exposed + f3_exposed + f4_exposed;
                if (num_exposed == 4) {
                    toErase.push_back(j);
                }
            }
        }
        if (toErase.size() > 0) {
            std::sort(toErase.begin(), toErase.end(), std::greater<int>());
            for (int j = 0; j < toErase.size(); j++) {
                auto it = printable_volumes[i].begin() + toErase[j];
                printable_volumes[i].erase(it);
            }
        }
    }
}

void MeshOperations::boolOpsApply(std::vector<std::vector<Eigen::Vector4i>> &printable_volumes, std::vector<Eigen::MatrixXf> &printable_vertices, std::vector<Eigen::MatrixXi> &printable_faces) {
    // step 0: Fix any flipped triangles prior to starting
    // TODO - doesn't seem to impact results, so not doing it.
    // print out the number of volumes
    // std::cout << "Number of volumes to do Bool Ops: " << printable_volumes.size() << std::endl;

    // Compute inset and cache it
    Eigen::MatrixXi insetF;
    Eigen::MatrixXd insetV;

    Eigen::MatrixXd normals;
    igl::per_vertex_normals(_V.cast<double>(), _F, normals);
    // pictures in slack with 0.15
    insetV = _V.cast<double>() - (_inset_amount * normals);
    insetF = _F; // same topology

    printable_vertices.resize(printable_volumes.size());
    printable_faces.resize(printable_volumes.size());

    // step 1: convert each volume into a surface mesh -- defined by a _F and _V collection
    for (int i = 0; i < printable_volumes.size(); i++) {
        std::vector<Eigen::Vector4i> volume = printable_volumes[i];

        // transfer contents of volume into a matrixXi
        Eigen::MatrixXi _currTT;
        _currTT.resize(volume.size(), 4);
        for (int j = 0; j < volume.size(); j++) {
            _currTT.row(j) = volume[j];
        }
        Eigen::MatrixXi newF;
        igl::boundary_facets(_currTT, newF);

        // Solid components simply convert the boundary facets to a mesh
        if (_solid_components) {
            Eigen::VectorXi I;
            // Remove unnecessary vertices (interior) to generate the boundary values
            igl::remove_unreferenced(_TV.cast<float>(), newF, printable_vertices[i], printable_faces[i], I);
        }

        // Hollow components requires subtracting out inset surface
        else {
            // Perform the boolean operation
            Eigen::VectorXi _I;

            newF.col(1).swap(newF.col(2));
            bool debug = igl::copyleft::cgal::mesh_boolean(
                 _TV, newF,
                insetV, insetF, igl::copyleft::cgal::string_to_mesh_boolean_type("minus"), printable_vertices[i], printable_faces[i], _I);
        }

        if (!_fabricate_skip_visualization) {
            // debug visualizer
            igl::opengl::glfw::Viewer viewer;
            // Set the mesh data
            viewer.data().set_mesh(printable_vertices[i].cast<double>(), printable_faces[i]);
            Eigen::RowVector3d mesh_color(0.8, 0.5, 0.2);  // orange color
            viewer.data().set_colors(mesh_color);
            viewer.launch();
        }
    }
}

// Writes components to an obj file
void MeshOperations::outputComponentsToFile(const std::vector<Eigen::MatrixXf> &printable_vertices, const std::vector<Eigen::MatrixXi> &printable_faces, const std::vector<Eigen::Vector3f> &printing_directions, const std::string &filename) {
    assert(printable_vertices.size() == printable_faces.size());
    for (int printable_component = 0; printable_component < printable_vertices.size(); printable_component++) {
        // Rotate printable component in the correct direction
        Eigen::MatrixXf rotated_vertices;
        rotated_vertices.resize(printable_vertices[printable_component].rows(), 3);
        Eigen::Vector3f printing_direction = printing_directions[printable_component];
        // This matrix will rotate the printing direction to directly upwards, so it will rotate the mesh so the printing direction is upwards
        Eigen::Matrix3f rotation = igl::rotation_matrix_from_directions(printing_direction, Eigen::Vector3f(0.0, 1.0, 0.0)).transpose();
        rotated_vertices = printable_vertices[printable_component] * rotation;

        // Produce filename
        std::string curr_filename = filename + "_" + std::to_string(printable_component) + ".obj";

        // Write to file
        igl::writeOBJ(curr_filename, rotated_vertices, printable_faces[printable_component]);
        std::cout << "Wrote printable component: " << printable_component << " to file " << curr_filename << std::endl;
    }
}


void MeshOperations::updateFaceMap(std::unordered_map<Vector3i, Vector4i, Vector3iHash, Vector3iEqual>& faceMap, Vector3i face, Vector4i tet) {
    if (faceMap.contains(face)) {
        faceMap.erase(face);
    } else {
        faceMap.insert({face, tet});
    }
}

Vector3i MeshOperations::orderVertices(Vector3i& face, Vector4i& tet) {
    // checking of v0, v1, v2 is in CCW order
    Vector3d v0Coords = _TV.row(face[0]);
    Vector3d v1Coords = _TV.row(face[1]);
    Vector3d v2Coords = _TV.row(face[2]);
    Vector3d normal = ((v1Coords - v0Coords).cross(v2Coords - v0Coords)).normalized();
    double d = -1 * (v0Coords).dot(normal);

    Vector3d tetVertex1 = _TV.row(tet[0]);
    Vector3d tetVertex2 = _TV.row(tet[1]);
    Vector3d tetVertex3 = _TV.row(tet[2]);
    Vector3d tetVertex4 = _TV.row(tet[3]);
    Vector3d tetCenter = (tetVertex1 + tetVertex2 + tetVertex3 + tetVertex4) / 4.f;
    Vector4d u = {tetCenter[0], tetCenter[1], tetCenter[2], 1}; // u should be centroid of tetrahedron

    Vector4d v{normal[0], normal[1], normal[2], d};
    if (u.dot(v) < 0) {
        return {face[0], face[1], face[2]};
    } else {
        // otherwise this should be CCW order
        v0Coords = _TV.row(face[1]);
        v1Coords = _TV.row(face[0]);
        v2Coords = _TV.row(face[2]);
        normal = ((v1Coords - v0Coords).cross(v2Coords - v0Coords)).normalized();
        d = -1 * (v0Coords).dot(normal);
        v = {normal[0], normal[1], normal[2], d};
        assert(u.dot(v) < 0);
        return {face[1], face[0], face[2]};
    }
}

void MeshOperations::extractSurface(const std::vector<Eigen::Vector4i> &volume, std::vector<Eigen::Vector3i> &surface_faces) {
    // for each tetrahedron
    // get the four faces
    // determine whether the face has already appeared
    std::unordered_map<Vector3i, Vector4i, Vector3iHash, Vector3iEqual> faceMap;
    for (int i = 0; i < volume.size(); i++) {
        Vector3i f0{volume[i][1], volume[i][2], volume[i][3]};
        Vector3i f1{volume[i][0], volume[i][2], volume[i][3]};
        Vector3i f2{volume[i][0], volume[i][1], volume[i][3]};
        Vector3i f3{volume[i][0], volume[i][1], volume[i][2]};
        updateFaceMap(faceMap, f0, volume[i]);
        updateFaceMap(faceMap, f1, volume[i]);
        updateFaceMap(faceMap, f2, volume[i]);
        updateFaceMap(faceMap, f3, volume[i]);
    }

    surface_faces.clear();
    for (auto it = faceMap.begin(); it != faceMap.end(); ++it) {
        Vector3i f = it->first;
        Vector4i tet = it->second;
        Vector3i ordered = orderVertices(f, tet);
        surface_faces.push_back(ordered);
    }
}



