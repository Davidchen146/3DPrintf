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
    void preprocess();
    void geodesicDistance();
    void angularDistance();
    void weightedDistance();
    void calculateAvgDistances();
    void makeAdjacency();
    double getGeodesicDistance(int i, int j);
    double getWeightedDistance(int i, int j);
    Eigen::VectorXd dijkstra(int start);


private:
    Mesh _mesh;
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
    std::vector<std::vector<bool>> _adjacency;
    MatrixXf _V;
    MatrixXi _F;
    Eigen::MatrixXd _geodesicDistances;
    Eigen::MatrixXd _angularDistances;
    Eigen::MatrixXd _weightedDistances;

    int _n;
    double _delta;
    double _avgGeodesic;
    double _avgAngular;
};

#endif // MESHOPERATIONS_H
