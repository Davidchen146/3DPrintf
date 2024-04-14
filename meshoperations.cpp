#include "meshoperations.h"
#include <igl/opengl/glfw/Viewer.h>

MeshOperations::MeshOperations(Mesh m) {
    _mesh = m;
    _vertices = _mesh.getVertices();
    _faces = _mesh.getFaces();
    numVertices = _vertices.size();
    numFaces = _faces.size();
    _V.resize(numVertices, 3);
    _F.resize(numFaces, 3);
    for (int i = 0; i < numVertices; i++) {
        _V.row(i) = _vertices[i];
    }
    for (int i = 0; i < numFaces; i++) {
        _F.row(i) = _faces[i];
    }
    _geodesicDistances.resize(numFaces, numFaces);
    _angularDistances.resize(numFaces, numFaces);
    _angularDistances.setZero();
}

void MeshOperations::geodesicDistance() {
    int n = _faces.size();
    VectorXi VS, VT, FS, FT;
    Eigen::VectorXd d;
    FS.resize(1);
    FT.resize(n);
    FT = VectorXi::LinSpaced(n, 0, n-1);
    for (int i = 0; i < n; i++) {
        d.setZero();
        FS[0] = i;
        igl::exact_geodesic(_V, _F, VS, FS, VT, FT, d);
        VectorXd d_copy = d;
        _geodesicDistances.row(i) = d_copy;
    }

    // std::cout << "Size: " << d.size() << std::endl;
    std::cout << _geodesicDistances << std::endl;
}

void MeshOperations::angularDistance() {
    unordered_set<Face *> faceSet = _mesh.getFaceSet();
    for (Face *f_i : faceSet) {
        Vector3f normal_i = f_i->normal;
        Halfedge *h = f_i->halfedge;
        // i think it's easier to do it this way so we know the edge the two adjacent faces share
        do {
            Face *f_j = h->twin->face;
            Vector3f normal_j = f_j->normal;

            // angle between face normals
            float alpha_ij = acos((normal_i.dot(normal_j)) / (normal_i.norm() * normal_j.norm()));

            // determine convexity / concavity
            Vertex *v1 = h->next->destination;
            Vertex *v2 = h->twin->next->destination;

            // midpoint between v1 & v2
            Vector3f midpoint_line = (v1->p + v2->p) / 2;

            // midpoint of halfedge h
            Vector3f midpoint_halfedge = (h->source->p + h->destination->p) / 2;
            Vector3f lineToHalfedge = midpoint_halfedge - midpoint_line;

            float n = 0.05; // convex
            // note: would this dot product ever be 0?
            if (lineToHalfedge.dot(normal_i) < 0) {
                assert(lineToHalfedge.dot(normal_j) < 0);
                // concave
                // std::cout << "concave" << std::endl;
                n = 1;
            }
            _angularDistances(f_i->index, f_j->index) = n * (1 - alpha_ij);
            h = h->next;
        } while (h != f_i->halfedge);
    }
    // std::cout << _angularDistances << std::endl;
}

void MeshOperations::visualize(vector<vector<int>>& coloringGroups) {
    // generate a certain number of colors based on coloringGroups
    Eigen::MatrixXd C;
    C.resize(_faces.size(), 3);

    std::unordered_map<int, int> faceToGroup;
    for (int i = 0; i < coloringGroups.size(); i++) {
        vector<int> group_i = coloringGroups[i];
        for (int j = 0; j < group_i.size(); j++) {
            int face_index = group_i[j];
            assert(!faceToGroup.contains(face_index));
            faceToGroup[face_index] = i;
        }
    }

    std::unordered_map<int, Vector3d> groupToColor;
    for (int i = 0; i < coloringGroups.size(); i++) {
        double m_red = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double m_green = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        double m_blue = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        Vector3d RGB = {m_red, m_green, m_blue};
        groupToColor[i] = RGB;
    }

    for (int i = 0; i < _faces.size(); i++) {
        assert(faceToGroup.contains(i));
        assert(groupToColor.contains(faceToGroup[i]));
        C.row(i) = groupToColor[faceToGroup[i]];
    }

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(_V.cast<double>(), _F);
    viewer.data().set_colors(C);
    viewer.launch();
}
