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
    void geodesicDistance();
    void angularDistance();
    double getGeodesicDistance(int i, int j);
    double getWeightedDistance(int i, int j);

private:
    Mesh _mesh;
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
    MatrixXf _V;
    MatrixXi _F;
    Eigen::MatrixXd _geodesicDistances;
    Eigen::MatrixXd _angularDistances;
};

#endif // MESHOPERATIONS_H
