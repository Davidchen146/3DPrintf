#ifndef MESHOPERATIONS_H
#define MESHOPERATIONS_H

#include <mesh.h>
#include "Eigen/StdVector"

#include <igl/exact_geodesic.h>
#include <Eigen/Core>

#include <unordered_map>
#include <limits>

using namespace Eigen;
using namespace std;

class MeshOperations
{

public:
    MeshOperations(Mesh m);

    // Computes global shortest path between faces using geodesic distance
    void preprocess();
    void geodesicDistance();

    // Computes local angular distance between pairs of adjacent faces
    void angularDistance();

    // Distances between arbitrary faces
    void visualize(std::vector<std::vector<int>>& coloringGroups);

    void weightedDistance();
    void calculateAvgDistances();
    void makeAdjacency();
    double getGeodesicDistance(int i, int j);
    double getWeightedDistance(int i, int j);
    Eigen::VectorXd dijkstra(int start);


    // Oversegmentation: returns list of lists of faces
    // Each list of faces represents a connected patch (to be merged and assigned a printing direction)
    void generateOversegmentation(std::vector<std::vector<int>> &patches);

private:
    Mesh _mesh;
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
  
    int numVertices;
    int numFaces;
  
    std::vector<std::vector<bool>> _adjacency;

    MatrixXf _V;
    MatrixXi _F;
    Eigen::MatrixXd _geodesicDistances;
    Eigen::MatrixXd _angularDistances;

    // Subroutines used for Phase 1 (Oversegmentation)
    // Initial seed computation
    void sampleRandomFaces(std::vector<int> &faces, int n);
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
    double bbd; // bounding box diagonal
    Eigen::MatrixXd _weightedDistances;

    int _n;
    double _delta;
    double _avgGeodesic;
    double _avgAngular;
};

#endif // MESHOPERATIONS_H
