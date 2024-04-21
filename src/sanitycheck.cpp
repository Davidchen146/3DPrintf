#include "src/meshoperations.h"
#include <igl/opengl/glfw/Viewer.h>

double max_ao;

// Linear interpolation function
Eigen::Vector3d lerp(const Eigen::Vector3d& color1, const Eigen::Vector3d& color2, double t) {
    return color1 + t * (color2 - color1);
}

Eigen::Vector3d mapAOToColor(double ao_value) {
    // Interpolate between blue (low AO), green/yellow (medium AO), and red (high AO)
    if (ao_value <= (max_ao / 2)) {
        // Blue to green/yellow interpolation
        return lerp(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 1.0, 0.0), ao_value * 2 / max_ao);
    } else {
        // Green/yellow to red interpolation
        return lerp(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), (ao_value - (max_ao / 2)) / (max_ao / 2));
    }
}

void MeshOperations::visualizeFaceAO() {
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);

    max_ao = 0;
    vector<double> ao_vals;
    for (int i = 0; i < _faces.size(); i++) {
        double ao = getFaceAO(i);
        max_ao = max(ao, max_ao);
        ao_vals.push_back(ao);
    }

    for (int i = 0; i < _faces.size(); i++) {
        C.row(i) = mapAOToColor(max_ao - ao_vals[i]);
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}

void MeshOperations::visualizeEdgeAO() {
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);

    max_ao = 0;
    vector<double> ao_vals;
    for (int i = 0; i < _faces.size(); i++) {
        Eigen::Vector3i vertexIndices = _faces[i];
        std::pair<int, int> e1 = _mesh.getSortedPair(vertexIndices[0], vertexIndices[1]);
        std::pair<int, int> e2 = _mesh.getSortedPair(vertexIndices[1], vertexIndices[2]);
        std::pair<int, int> e3 = _mesh.getSortedPair(vertexIndices[0], vertexIndices[2]);
        double ao1 = getEdgeAO(e1);
        double ao2 = getEdgeAO(e2);
        double ao3 = getEdgeAO(e3);
        double ao = (ao1 + ao2 + ao3) / 3;
        max_ao = max(ao, max_ao);
        ao_vals.push_back(ao);
    }

    for (int i = 0; i < _faces.size(); i++) {
        C.row(i) = mapAOToColor(max_ao - ao_vals[i]);
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}


