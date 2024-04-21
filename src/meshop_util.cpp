#include "src/meshoperations.h"
#include <limits.h>

// Computes normal for a face
Eigen::Vector3f MeshOperations::getFaceNormal(const int &face) {
    Eigen::Vector3f faceNormal = _mesh.getFace(face)->normal;
    return faceNormal;
}

// Normal computation for edge
Eigen::Vector3f MeshOperations::getEdgeNormal(const std::pair<int, int> &edge) {
    Eigen::Vector3f edgeNormal = _mesh.getEdge(edge)->normal;
    return edgeNormal;
}

// Normal computation for vertex
Eigen::Vector3f MeshOperations::getVertexNormal(const int &vertex) {
    Eigen::Vector3f vertexNormal = _mesh.getVertex(vertex)->normal;
    return vertexNormal;
}

// Computes area for a face
// Implements half cross product formula from: https://math.stackexchange.com/questions/128991/how-to-calculate-the-area-of-a-3d-triangle
double MeshOperations::getArea(const int &face) {
    assert(face >= 0 && face < _faces.size());

    // Obtain 3 vertices of the face
    Eigen::Vector3i face_vertices = _faces[face];
    Eigen::Vector3f A = _vertices[face_vertices(0)];
    Eigen::Vector3f B = _vertices[face_vertices(1)];
    Eigen::Vector3f C = _vertices[face_vertices(2)];

    // Create the edges that define the cross product (these lie on the plane of the face)
    Eigen::Vector3f AB = B - A;
    Eigen::Vector3f AC = C - A;

    // Do the cross product (this is a normal vector to the face)
    Eigen::Vector3f cross = AC.cross(AB);
    return cross.norm() / 2;
}

// Compute the centroid for a face
Eigen::Vector3f MeshOperations::getCentroid(const int &face) {
    assert(face >= 0 && face < _faces.size());

    // Obtain 3 vertices of the face
    Eigen::Vector3i face_vertices = _faces[face];
    Eigen::Vector3f A = _vertices[face_vertices(0)];
    Eigen::Vector3f B = _vertices[face_vertices(1)];
    Eigen::Vector3f C = _vertices[face_vertices(2)];

    // Centroid is their average lul
    return (A + B + C) / 3;
}

// Returns smallest geodesic distance from queried face to faces in set
std::pair<double, int> MeshOperations::getMinGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces, bool include_self) {
    double min_distance = std::numeric_limits<double>::max();
    int min_face = -1;

    // FInd smallest distance by iterating over all faces in set
    for (const int &face_in_set : faces) {
        // Only include same faces if include_self is checked (param control this)
        if (!include_self && face_in_set == face) {
            continue;
        }

        double current_distance = getGeodesicDistance(face, face_in_set);
        if (current_distance < min_distance) {
            min_distance = current_distance;
            min_face = face_in_set;
        }
    }

    return std::make_pair(min_distance, min_face);
}

// Returns smallest weighted distance from queried faces to faces in set
// Weighted distance is linear combination of geodesic and angular distances
std::pair<double, int> MeshOperations::getMinWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces, bool include_self) {
    double min_distance = std::numeric_limits<double>::max();
    int min_face = -1;

    // FInd smallest distance by iterating over all faces in set
    for (const int &face_in_set : faces) {
        // Only include same faces if include_self is checked (param control this)
        if (!include_self && face_in_set == face) {
            continue;
        }

        double current_distance = getWeightedDistance(face, face_in_set);
        if (current_distance < min_distance) {
            min_distance = current_distance;
            min_face = face_in_set;
        }
    }

    return std::make_pair(min_distance, min_face);
}

// Returns smallest geodesic distance from queried face to faces in set
std::pair<double, int> MeshOperations::getMaxGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces) {
    double max_distance = -1;
    int max_face = -1;

    // FInd smallest distance by iterating over all faces in set
    for (const int &face_in_set : faces) {
        double current_distance = getGeodesicDistance(face, face_in_set);
        if (current_distance > max_distance) {
            max_distance = current_distance;
            max_face = face_in_set;
        }
    }

    return std::make_pair(max_distance, max_face);
}

// Returns smallest weighted distance from queried faces to faces in set
// Weighted distance is linear combination of geodesic and angular distances
std::pair<double, int> MeshOperations::getMaxWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces) {
    double max_distance = -1;
    int max_face = -1;

    // FInd smallest distance by iterating over all faces in set
    for (const int &face_in_set : faces) {
        double current_distance = getWeightedDistance(face, face_in_set);
        if (current_distance > max_distance) {
            max_distance = current_distance;
            max_face = face_in_set;
        }
    }

    return std::make_pair(max_distance, max_face);
}

// Returns total geodesic distance from queried face to faces in set
double MeshOperations::getTotalGeodesicDistanceToSet(const int &face, const std::unordered_set<int> &faces) {
    double total_dist = 0;
    // Add up distance from all faces
    for (const int &face_in_set : faces) {
        total_dist += getGeodesicDistance(face, face_in_set);
    }

    return total_dist;
}

// Returns total weighted distance from queried face to faces in set
double MeshOperations::getTotalWeightedDistanceToSet(const int &face, const std::unordered_set<int> &faces) {
    double total_dist = 0;
    // Add up distance from all faces
    for (const int &face_in_set : faces) {
        total_dist += getWeightedDistance(face, face_in_set);
    }

    return total_dist;
}

// For random direction generation
Eigen::Vector3f MeshOperations::generateRandomVector() {
    // randomly sample from the sphere
    float phi = 2.f * std::numbers::pi * (float) rand() / (float) UINT32_MAX;
    float theta = acos(2 * ((float) rand() / (float) UINT32_MAX) - 1);

    // spherical to rectangular converion (with radius = 1)
    float x = sin(theta) * cos(phi);
    float y = cos(theta);
    float z = sin(theta) * sin(phi);
    Vector3f direction(x, y, z);
    return direction;
}

// Sample a random point on a face
Eigen::Vector3f MeshOperations::sampleRandomPoint(const int &face) {
    // Draw two random numbers
    float r1 = rand() / (float) RAND_MAX;
    float r2 = rand() / (float) RAND_MAX;

    float v1_coeff = 1 - sqrt(r1);
    float v2_coeff = sqrt(r1) * (1 - r2);
    float v3_coeff = r2 * sqrt(r1);

    // These are barycentric coordinates used to interpolate points on the triangle
    return (v1_coeff * _V.row(_F.row(face)(0))) + (v2_coeff * _V.row(_F.row(face)(1))) + (v3_coeff * _V.row(_F.row(face)(2)));
}

// For determining intersections with other faces
// Should use BVH, some other structure, or there might be something in libigl/VCGlib we can use
// If using BVH, may need to make BVH initialization a preprocessing step
// It should return the face it intersects
// Returns -1 if no face was intersected
int MeshOperations::getIntersection(const Eigen::Vector3f &ray_position, const Eigen::Vector3f &ray_direction) {
    // Invoke the Embree raytracer, baby!
    igl::Hit hit;
    if (_intersector.intersectRay(ray_position, ray_direction, hit)) {
        // This is the ID (number) of the intersected face
        return hit.id;
    }
    // -1 Means that no face was hit (duh)
    return -1;
}

// Gets intersection of edges (or faces, TBD) between two patches
void MeshOperations::getBoundaryEdges(const std::unordered_set<int> &patch_one, const std::unordered_set<int> &patch_two, std::unordered_set<std::pair<int, int>> &boundaryEdges) {
    return;
}

// Ambient occlusion operation (edge)
double MeshOperations::getEdgeAO(const std::pair<int, int> &edge) {
    // Initialize params to for libigl call (see https://github.com/libigl/libigl/blob/main/tutorial/606_AmbientOcclusion/main.cpp)
    Eigen::Matrix <float, 2, 3> edge_coords;
    Eigen::Matrix <float, 2, 3> edge_normals;
    Eigen::Vector2d AO;

    edge_coords.row(0) = _V.row(edge.first);
    edge_coords.row(1) = _V.row(edge.second);
    edge_normals.row(0) = getVertexNormal(edge.first);
    edge_normals.row(1) = getVertexNormal(edge.second);

    igl::embree::ambient_occlusion(_V, _F, edge_coords, edge_normals, _ambient_occlusion_samples, AO);
    return AO.mean();
}

// Ambient occlusion operation (face)
double MeshOperations::getFaceAO(const int &face) {
    // Initialize params to for libigl call (see https://github.com/libigl/libigl/blob/main/tutorial/606_AmbientOcclusion/main.cpp)
    Eigen::Matrix <float, 3, 3> face_coords;
    Eigen::Matrix <float, 3, 3> face_normals;
    Eigen::Vector3d AO;

    face_coords.row(0) = _V.row(_F.row(face)(0));
    face_coords.row(1) = _V.row(_F.row(face)(1));
    face_coords.row(2) = _V.row(_F.row(face)(2));
    face_normals.row(0) = getVertexNormal(_F.row(face)(0));
    face_normals.row(1) = getVertexNormal(_F.row(face)(1));
    face_normals.row(1) = getVertexNormal(_F.row(face)(2));

    igl::embree::ambient_occlusion(_V, _F, face_coords, face_normals, _ambient_occlusion_samples, AO);
    return AO.mean();
}
