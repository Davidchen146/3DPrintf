#include "src/meshoperations.h"
#include <limits.h>

// Computes normal for a face
Eigen::Vector3f MeshOperations::getNormal(const int &face) {
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
    return cross.normalized();
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
std::pair<double, int> MeshOperations::getMinGeodesicDistanceToSet(const int &face, const std::vector<int> &faces, bool include_self) {
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
std::pair<double, int> MeshOperations::getMinWeightedDistanceToSet(const int &face, const std::vector<int> &faces, bool include_self) {
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
std::pair<double, int> MeshOperations::getMaxGeodesicDistanceToSet(const int &face, const std::vector<int> &faces) {
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
std::pair<double, int> MeshOperations::getMaxWeightedDistanceToSet(const int &face, const std::vector<int> &faces) {
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
double MeshOperations::getTotalGeodesicDistanceToSet(const int &face, const std::vector<int> &faces) {
    double total_dist = 0;
    // Add up distance from all faces
    for (const int &face_in_set : faces) {
        total_dist += getGeodesicDistance(face, face_in_set);
    }

    return total_dist;
}

// Returns total weighted distance from queried face to faces in set
double MeshOperations::getTotalWeightedDistanceToSet(const int &face, const std::vector<int> &faces) {
    double total_dist = 0;
    // Add up distance from all faces
    for (const int &face_in_set : faces) {
        total_dist += getWeightedDistance(face, face_in_set);
    }

    return total_dist;
}
