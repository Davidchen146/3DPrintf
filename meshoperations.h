#ifndef MESHOPERATIONS_H
#define MESHOPERATIONS_H

#include <mesh.h>
#include "Eigen/StdVector"

#include <igl/exact_geodesic.h>
#include <Eigen/Core>

#include <unordered_map>

using namespace Eigen;
using namespace std;

class MeshOperations
{

public:
    MeshOperations(Mesh m);

    // Computes global shortest path between faces using geodesic distance
    void geodesicDistance();

    // Computes local angular distance between pairs of adjacent faces
    void angularDistance();

    // Distances between arbitrary faces
    double getGeodesicDistance(int i, int j);
    double getWeightedDistance(int i, int j);

    // Oversegmentation: returns list of lists of faces
    // Each list of faces represents a connected patch (to be merged and assigned a printing direction)
    void generateOversegmentation(std::vector<std::vector<int>> &patches);

private:
    Mesh _mesh;
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
    MatrixXf _V;
    MatrixXi _F;
    Eigen::MatrixXd _geodesicDistances;
    Eigen::MatrixXd _angularDistances;

    // Subroutines used for Phase 1 (Oversegmentation)
    // Initial seed computation
    void sampleRandomFaces(std::vector<int> &faces);
    void generateInitialSeeds(std::vector<int> &seeds);

    // Iterative steps to generate oversegmentation
    void generatePatches(const std::vector<int> &seeds, std::vector<std::vector<int>> &patches);
    void recenterSeeds(const std::vector<std::vector<int>> &patches, std::vector<int> &new_seeds);

    // Basic utility functions
    Eigen::Vector3f getNormal(const int &face);
    Eigen::Vector3f getCentroid(const int &face);
    double getArea(const int &face);
    std::pair<double, int> getGeodesicDistanceToSet(const int &face, const std::vector<int> &faces, bool include_self = false);
    std::pair<double, int> getWeightedDistanceToSet(const int &face, const std::vector<int> &faces, bool include_self = false);
};

#endif // MESHOPERATIONS_H
