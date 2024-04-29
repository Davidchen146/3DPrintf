#include "meshoperations.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>

int minDistanceVertex(vector<double> distances, vector<bool> visited) {
    double minDistance = std::numeric_limits<double>::infinity();
    int v = -1;
    for (int i = 0; i < distances.size(); i++) {
        if (!visited[i] && distances[i] < minDistance) {
            minDistance = distances[i];
            v = i;
        }
    }
    return v;
}

VectorXd MeshOperations::dijkstra(int start) {
    vector<double> distances;
    vector<bool> visited;
    VectorXd d;
    // initialize distances and visited vectors
    for (int i = 0; i < _n; i++) {
        distances.push_back(std::numeric_limits<double>::infinity());
        visited.push_back(false);
    }
    distances[start] = 0;

    for (int i = 0; i < _n; i++) {
        int u = minDistanceVertex(distances, visited);
        if (u == -1) {
            // if we are done with the algorithm
            break;
        }
        visited[u] = true;

        for (int v = 0; v < _n; v++) {
            if (_adjacency[u][v] && !visited[v]) {
                double altDistance = distances[u] + (_geodesic_distance_weight*(getGeodesicDistance(u, v) / _avgGeodesic) + ((1.0 - _geodesic_distance_weight)*(_angularDistances(u, v) / _avgAngular)));
                if (altDistance < distances[v]) {
                    distances[v] = altDistance;
                }
            }
        }
    }

    d.resize(_n);
    d.setZero();
    for (int i = 0; i < _n; i++) {
        d[i] = distances[i];
    }
    return d;
}

// Preprocessing subroutines
void MeshOperations::preprocessData() {
    std::cout << "Loading mesh data" << std::endl;
    _vertices = _mesh.getVertices();
    _faces = _mesh.getFaces();
    numVertices = _vertices.size();
    numFaces = _faces.size();
    _n = numFaces;
    _V.resize(numVertices, 3);
    _F.resize(numFaces, 3);
    for (int i = 0; i < numVertices; i++) {
        _V.row(i) = _vertices[i];
    }
    for (int i = 0; i < numFaces; i++) {
        _F.row(i) = _faces[i];
    }
}

void MeshOperations::makeAdjacency() {
    std::vector<std::vector<bool>> matrix(_n, std::vector<bool>(_n, false));
    _adjacency = matrix;

    unordered_map<int, Face *> faceMap = _mesh.getFaceMap();
    for (const auto& pair : faceMap) {
        Face *f = pair.second;
        for (Face* n: f->neighbors) {
            _adjacency[f->index][n->index] = true;
        }
    }
}

// Does preprocessing related to mesh distance computations
// Must be called after data is preprocessed in preprocessData
void MeshOperations::preprocessDistances() {
    _geodesicDistances.resize(numFaces, numFaces);
    _angularDistances.resize(numFaces, numFaces);
    _weightedDistances.resize(numFaces, numFaces);

    _geodesicDistances.setZero();
    _angularDistances.setZero();
    bbd = (_V.colwise().maxCoeff()- _V.colwise().minCoeff()).norm();

    _weightedDistances.setZero();
    makeAdjacency();
    std::cout << "Performing geodesic distance computations" << std::endl;
    geodesicDistance();
    std::cout << "Performing angular distance computations" << std::endl;
    angularDistance();
    calculateAvgDistances();
    weightedDistance();
    assert(_weightedDistances.isApprox(_weightedDistances.transpose()));
}

// Does preprocessing related to raytracing operations
// Must be called after data is preprocessed in preprocessData
void MeshOperations::preprocessRaytracer() {
    // Prepare raytracer
    std::cout << "Loading mesh into Embree raytracer" << std::endl;
    _intersector.init(_V, _F);
}

void MeshOperations::geodesicDistance() {
    int n = _faces.size();
    VectorXi VS, VT, FS, FT;
    Eigen::VectorXd d;
    FS.resize(1);
    FT.resize(n);
    FT = VectorXi::LinSpaced(n, 0, n-1);
    for (int i = 0; i < n; i++) {
        d.setZero();
        FS[0] = i;
        igl::exact_geodesic(_V, _F, VS, FS, VT, FT, d);
        VectorXd d_copy = d;
        _geodesicDistances.row(i) = d_copy;
    }
}

void MeshOperations::angularDistance() {
    unordered_map<int, Face *> faceMap = _mesh.getFaceMap();
    for (const auto& pair : faceMap) {
        Face *f_i = pair.second;
        Vector3f normal_i = f_i->normal;
        Halfedge *h = f_i->halfedge;
        // i think it's easier to do it this way so we know the edge the two adjacent faces share
        do {
            Face *f_j = h->twin->face;
            Vector3f normal_j = f_j->normal;

            // angle between face normals
            float cos_alpha_ij = (normal_i.dot(normal_j)) / (normal_i.norm() * normal_j.norm());

            // determine convexity / concavity
            Vertex *v1 = h->next->destination;
            Vertex *v2 = h->twin->next->destination;

            // midpoint between v1 & v2
            Vector3f midpoint_line = (v1->p + v2->p) / 2;

            // midpoint of halfedge h
            Vector3f midpoint_halfedge = (h->source->p + h->destination->p) / 2;
            Vector3f lineToHalfedge = midpoint_halfedge - midpoint_line;

            float n = _convex_coeff; // convex
            // note: would this dot product ever be 0?
            if (lineToHalfedge.dot(normal_i) < 0) {
                assert(lineToHalfedge.dot(normal_j) < 0);
                // concave
                n = _concave_coeff;
            }
            _angularDistances(f_i->index, f_j->index) = n * (1 - cos_alpha_ij);
            h = h->next;
        } while (h != f_i->halfedge);
    }
}

void MeshOperations::weightedDistance() {
    for (int i = 0; i < _n; i++) {
        VectorXd distances = dijkstra(i);
        _weightedDistances.row(i) = distances;
    }
}

void MeshOperations::calculateAvgDistances() {
    int numPairs = 0;
    double sumAngular = 0;
    double sumGeodesic = 0;

    for (int i = 0; i < _n; i++) {
        for (int j = 0; j < _n; j++) {
            if (_adjacency[i][j]) {
                // If this is a connected pair
                numPairs += 1;
                sumAngular += getAngularDistance(i, j);
                sumGeodesic += getGeodesicDistance(i, j);
            }
        }
    }

    _avgGeodesic = sumGeodesic / numPairs;
    _avgAngular = sumAngular / numPairs;
}

// Preprocess to select faces that we will not count support costs for
void MeshOperations::preprocessZeroCostFaces() {
    // Does nothing if zero cost set is not enabled
    if (!_use_zero_cost_faces) {
        return;
    }

    // Use viewer to highlight faces that shouldn't be costed

    // Indicator variables
    bool clicked_on_mesh = false;
    bool adding_zero_cost_faces = false;
    bool removing_zero_cost_faces = false;

    // Colors to render which faces we've selected and deselected
    Eigen::Vector3d selected_color(150.0f / 255, 210.0f / 255, 148.0f / 255);
    Eigen::Vector3d unselected_color(65.0f / 255, 65.0f / 255, 65.0f / 255);

    // Initialize default colors
    Eigen::MatrixXd colors;
    colors.resize(_F.rows(), 3);
    for (int face = 0; face < _F.rows(); face++) {
        colors.row(face) = unselected_color;
    }

    // Mouse down, mouse up, and mouse move functions that interact with the set
    // When adding/removing a face, change the color we use to render that face
    igl::opengl::glfw::Viewer viewer;

    // Funny lambdas
    const auto select_face = [&]()->int {
        const double x = viewer.current_mouse_x;
        const double y = viewer.core().viewport(3) - viewer.current_mouse_y;
        int face_id;
        Eigen::Vector3f barycentric_coordinates;
        // If there's been a hit
        if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view, viewer.core().proj, viewer.core().viewport, _V, _F, face_id, barycentric_coordinates)) {
            // face_id contains the face we hit
            return face_id;
        }
        return -1;
    };

    // When mouse button clicked
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer &viewer, int, int)->bool {
        int selected_face = select_face();
        if (selected_face != -1) {
            clicked_on_mesh = true;
        }

        if (!clicked_on_mesh) {
            return false;
        }

        // If you clicked on the mesh, determine what operation we should do
        if (_zero_cost_faces.contains(selected_face)) {
            // Removing faces
            removing_zero_cost_faces = true;
            adding_zero_cost_faces = false;
        } else {
            // Adding faces
            adding_zero_cost_faces = true;
            removing_zero_cost_faces = false;
        }

        // Add face
        if (adding_zero_cost_faces) {
            _zero_cost_faces.insert(selected_face);
            colors.row(selected_face) = selected_color;
            viewer.data().set_colors(colors);
        }

        // Remove face
        if (removing_zero_cost_faces) {
            _zero_cost_faces.erase(selected_face);
            colors.row(selected_face) = unselected_color;
            viewer.data().set_colors(colors);
        }

        return true;
    };

    // When mouse button moved
    viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer &viewer, int, int)->bool {
        // Only add more faces if you've clicked on the mesh
        if (!clicked_on_mesh) {
            return false;
        }

        // Otherwise determine what mode you're in
        int selected_face = select_face();
        if (selected_face == -1) {
            return true;
        }

        if (adding_zero_cost_faces) {
            _zero_cost_faces.insert(selected_face);
            colors.row(selected_face) = selected_color;
            viewer.data().set_colors(colors);
        }

        if (removing_zero_cost_faces) {
            _zero_cost_faces.erase(selected_face);
            colors.row(selected_face) = unselected_color;
            viewer.data().set_colors(colors);
        }

        return true;
    };

    // When mouse button released
    viewer.callback_mouse_up = [&clicked_on_mesh](igl::opengl::glfw::Viewer &viewer, int, int)->bool {
        clicked_on_mesh = false;
        return false;
    };

    // Launch the viewer using the behavior we coded
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(colors);
    viewer.launch();
}
