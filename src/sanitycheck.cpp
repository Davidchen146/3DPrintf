#include "src/meshoperations.h"
#include <igl/opengl/glfw/Viewer.h>

void MeshOperations::visualizeFaceAO() {
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);

    double max_ao = 0;

    vector<double> ao_vals;
    for (int i = 0; i < _faces.size(); i++) {
        double ao = getFaceAO(i);
        max_ao = max(ao, max_ao);
        ao_vals.push_back(ao);
    }
    std::cout << "max_ao: " << max_ao << std::endl;

    // igl::opengl::glfw::Viewer viewer;
    // viewer.data().set_mesh(_V.cast<double>(), _F);
    // viewer.data().set_colors(C);
    // viewer.launch();
}

void MeshOperations::visualizeEdgeAO() {
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}


