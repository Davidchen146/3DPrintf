#include "meshoperations.h"

MeshOperations::MeshOperations(Mesh m) {
    _mesh = m;
    preprocess();
}

void MeshOperations::preprocess() {
    _vertices = _mesh.getVertices();
    _faces = _mesh.getFaces();
    int numVertices = _vertices.size();
    int numFaces = _faces.size();
    _n = numFaces;
    _delta = 0.1;
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
    _weightedDistances.setZero();

    makeAdjacency();
    geodesicDistance();
    angularDistance();
    calculateAvgDistances();
    weightedDistance();
    assert(_weightedDistances.isApprox(_weightedDistances.transpose()));
    std::cout << _weightedDistances << std::endl;
}

void MeshOperations::makeAdjacency() {
    std::vector<std::vector<bool>> matrix(_n, std::vector<bool>(_n, false));
    _adjacency = matrix;

    unordered_set<Face *> faceSet = _mesh.getFaceSet();
    for (Face* f: faceSet) {
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

    // std::cout << "Size: " << d.size() << std::endl;
    // std::cout << _geodesicDistances << std::endl;
}

void MeshOperations::angularDistance() {
    unordered_set<Face *> faceSet = _mesh.getFaceSet();
    for (Face *f_i : faceSet) {
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

            float n = 0.05; // convex
            // note: would this dot product ever be 0?
            if (lineToHalfedge.dot(normal_i) < 0) {
                assert(lineToHalfedge.dot(normal_j) < 0);
                // concave
                std::cout << "concave" << std::endl;
                n = 1;
            }
            // std::cout << cos_alpha_ij << std::endl;
            _angularDistances(f_i->index, f_j->index) = n * (1 - cos_alpha_ij);
            h = h->next;
        } while (h != f_i->halfedge);
    }
    // std::cout << _angularDistances << std::endl;
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

    return 1;
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
                double altDistance = distances[u] + (_delta*(getGeodesicDistance(u, v) / _avgGeodesic) + ((1.0 - _delta)*(_angularDistances(u, v) / _avgAngular)));
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
