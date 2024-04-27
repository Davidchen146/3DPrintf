#include "src/meshoperations.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>

// Mesh the interior of a surface mesh (V,F) using tetgen
// @param[in] V #V x 3 vertex position list
// @param[in] F #F x 3 indices into V
// @param[in] switches string of tetgen options e.g.
//     "pq1.414a0.01" tries to mesh the interior of a given surface with
//       quality and area constraints -- i have no idea what should be used here
// @param[out] TV #TV x 3 vertex position list
// @param[out] TT #TT x 4 list of tet face indices
// @param[out] TF #TF x 3 list of triangle face indices **only surface faces** (contents of TF are the same as _F)
void MeshOperations::tetrahedralizeMesh() {
    Eigen::MatrixXd TV;
    Eigen::MatrixXi TT;
    Eigen::MatrixXi TF; // TF not necessary? (but maybe helpful)
    igl::copyleft::tetgen::tetrahedralize(_V.cast<double>(), _F, "pq1.414Y", TV, TT, TF);

    // print dimensions
    std::cout << "TV dimensions: " << TV.rows() << " " << TV.cols() << std::endl;
    std::cout << "TT dimensions: " << TT.rows() << " " << TT.cols() << std::endl;
    std::cout << "trimesh faces: " << _F.rows() << " " << _F.cols() << std::endl;
    std::cout << "TF dimensions: " << TF.rows() << " " << TF.cols() << std::endl;

    std::ofstream fileF("matrixF.txt");
    std::ofstream fileTF("matrixTF.txt");

    if (fileTF.is_open() && fileF.is_open()) {
        fileF << _F << std::endl;
        fileF.close();
        fileTF << TF << std::endl;
        fileTF.close();
    }
}
