#include "src/meshoperations.h"
#include <igl/opengl/glfw/Viewer.h>

MeshOperations::MeshOperations(Mesh m) {
    _mesh = m;

    // Set default parameters for operations here
    // Preprocessing parameters
    _geodesic_distance_weight = 0.1;
    _convex_coeff = 0.05;
    _concave_coeff = 1;

    // Oversegmentation parameters
    _num_seed_faces = 0;
    _proportion_seed_faces = 0.1;
    _oversegmentation_bounding_box_coeff = 0.01;
    _num_oversegmentation_iterations = 3;
    _seeds_only = false;

    // Initial segmentation parameters
    _num_random_dir_samples = 512;
    _printer_tolerance_angle = 55.f / 180.f * std::numbers::pi; // 55 degrees in radians
    _ambient_occlusion_supports_alpha = 0.5;
    _ambient_occlusion_smoothing_alpha = 0.5;
    _smoothing_width_t = 0.3;
    _ambient_occlusion_samples = 500;
    _footing_samples = 1;
}

// Configure parameters for 3D printing operations
void MeshOperations::setPreprocessingParameters(double geodesic_weight,
                                                double convex_coeff,
                                                double concave_coeff) {
    // Check against default value (0) to prevent loading in unspecified parameters
    if (geodesic_weight != 0.0) {
        _geodesic_distance_weight = geodesic_weight;
    }

    if (convex_coeff != 0.0) {
        _convex_coeff = convex_coeff;
    }

    if (concave_coeff != 0.0) {
        _concave_coeff = concave_coeff;
    }
}

void MeshOperations::setOversegmentationParameters(int num_seed_faces,
                                                   double proportion_seed_faces,
                                                   double bounding_box_coeff,
                                                   int num_iterations,
                                                   bool seeds_only) {
    // Check against default value (0) to prevent loading in unspecified parameters
    if (num_seed_faces != 0.0) {
        _num_seed_faces = num_seed_faces;
    }

    if (proportion_seed_faces != 0.0) {
        _proportion_seed_faces = proportion_seed_faces;
    }

    if (bounding_box_coeff != 0.0) {
        _oversegmentation_bounding_box_coeff = bounding_box_coeff;
    }

    if (num_iterations != 0) {
        _num_oversegmentation_iterations = num_iterations;
    }

    if (seeds_only) {
        _seeds_only = seeds_only;
    }
}

void MeshOperations::setInitialSegmentationParameters(int num_random_dir_samples,
                                                      double printer_tolerance_angle,
                                                      double ambient_occlusion_supports_alpha,
                                                      double ambient_occlusion_smoothing_alpha,
                                                      double smoothing_width_t,
                                                      int ambient_occlusion_samples,
                                                      int footing_samples) {
    // Check against default value (0) to prevent loading in unspecified parameters
    if (num_random_dir_samples != 0) {
        _num_random_dir_samples = num_random_dir_samples;
    }

    if (printer_tolerance_angle != 0.0) {
        // Convert the parameter to radians
        double angle_rads = printer_tolerance_angle / 180.f * std::numbers::pi;
        _printer_tolerance_angle = angle_rads;
    }

    if (ambient_occlusion_supports_alpha != 0.0) {
        _ambient_occlusion_supports_alpha = ambient_occlusion_supports_alpha;
    }

    if (ambient_occlusion_smoothing_alpha != 0.0) {
        _ambient_occlusion_smoothing_alpha = ambient_occlusion_smoothing_alpha;
    }

    if (smoothing_width_t != 0.0) {
        _smoothing_width_t = smoothing_width_t;
    }

    if (ambient_occlusion_samples != 0) {
        _ambient_occlusion_samples = ambient_occlusion_samples;
    }

    if (footing_samples != 0) {
        _footing_samples = footing_samples;
    }
}

void MeshOperations::preprocess() {
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

    // Prepare raytracer
    std::cout << "Loading mesh into Embree raytracer" << std::endl;
    _intersector.init(_V, _F);
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

void MeshOperations::visualize(vector<unordered_set<int>>& coloringGroups) {
    // generate a certain number of colors based on coloringGroups
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);
    Vector3d black = {0, 0, 0};
    for (int i = 0; i < _faces.size(); i++) {
        C.row(i) = black;
    }

    std::unordered_map<int, int> faceToGroup;
    for (int i = 0; i < coloringGroups.size(); i++) {
        std::unordered_set<int> group_i = coloringGroups[i];
        for (auto j = group_i.begin(); j != group_i.end(); j++) {
            int face_index = *j;
            assert(!faceToGroup.contains(face_index));
            faceToGroup[face_index] = i;
        }
    }

    std::unordered_map<int, Vector3d> groupToColor;
    for (int i = 0; i < coloringGroups.size(); i++) {
        double m_red = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double m_green = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double m_blue = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        Vector3d RGB = {m_red, m_green, m_blue};
        groupToColor[i] = RGB;
    }

    for (int i = 0; i < _faces.size(); i++) {
        if (!_seeds_only) {
            // assert(faceToGroup.contains(i));
            // assert(groupToColor.contains(faceToGroup[i]));
        }
        if (faceToGroup.contains(i)) {
            C.row(i) = groupToColor[faceToGroup[i]];
        } else {
            C.row(i) = black;
        }
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
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
                sumAngular += _angularDistances(i, j);
                sumGeodesic += getGeodesicDistance(i, j);
            }
        }
    }

    _avgGeodesic = sumGeodesic / numPairs;
    _avgAngular = sumAngular / numPairs;
}

double MeshOperations::getGeodesicDistance(int i, int j) {
    return _geodesicDistances(i, j);
}

double MeshOperations::getWeightedDistance(int i, int j) {
    return _weightedDistances(i, j);
}

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
