#include "meshoperations.h"
#include <igl/blue_noise.h>

// Implments the Oversegmentation Routine and any necessary subroutines

// Generates oversegmentation
std::vector<std::vector<int>> MeshOperations::generateOversegmentation(){

}

void MeshOperations::sampleRandomFaces(std::vector<int> &faces, int n) {
    // n is *approximately* the number of faces you want to sample
    // but I can't get it such that it's always exactly the number of faces you want to sample
    // this is just a heuristic so modify however you wish
    Eigen::VectorXd A;
    igl::doublearea(_V, _F, A);
    double r = sqrt(A.sum()/(n * igl::PI));

    Eigen::MatrixXd B;
    Eigen::VectorXi FI; // indices into _F
    Eigen::MatrixXd P_blue;
    igl::blue_noise(_V,_F,r,B,FI,P_blue);

    // std::cout << FI.size() << std::endl;

    for (int i = 0; i < FI.size(); i++) {
        faces.push_back(FI[i]);
    }
}

void MeshOperations::generateInitialSeeds(std::vector<int> &seeds) {

}

// Iterative steps to generate oversegmentation
void MeshOperations::generatePatches(const std::vector<int> &seeds, std::vector<std::vector<int>> &patches) {

}

void MeshOperations::recenterSeeds(const std::vector<std::vector<int>> &patches, std::vector<int> &new_seeds) {

}
