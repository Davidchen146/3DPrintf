#include "meshoperations.h"

// Computes normal for a face
Eigen::Vector3f MeshOperations::getNormal(const int &face) {

}

// Computes area for a face
double MeshOperations::getArea(const int &face) {

}

// Returns smallest geodesic distance from queried face to faces in set
double MeshOperations::getGeodesicDistanceToSet(const int &face, const std::vector<int> &faces, bool include_self) {

}

// Returns smallest weighted distance from queried faces to faces in set
// Weighted distance is linear combination of geodesic and angular distances
double MeshOperations::getWeightedDistanceToSet(const int &face, const std::vector<int> &faces, bool include_self) {

}
