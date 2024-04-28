#include "src/meshoperations.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <QString>

// Mesh the interior of a surface mesh (V,F) using tetgen
// @param[in] V #V x 3 vertex position list
// @param[in] F #F x 3 indices into V
// @param[in] switches string of tetgen options, see https://wias-berlin.de/software/tetgen/switches.html
// @param[out] TV #TV x 3 vertex position list
// @param[out] TT #TT x 4 list of tet face indices
// @param[out] TF #TF x 3 list of triangle face indices
// (contents of TF are the same as _F --> IF "p" option is specified)
void MeshOperations::tetrahedralizeMesh() {
    // i just took the options from the example, not 100% what to use here
    igl::copyleft::tetgen::tetrahedralize(_V.cast<double>(), _F, "pq1.414Y", _TV, _TT, _TF);
}

Eigen::Vector3d MeshOperations::computeTetCentroid(Eigen::Vector4i &tetrahedron) {
    Vector3d tetVertex1 = _TV.row(tetrahedron[0]);
    Vector3d tetVertex2 = _TV.row(tetrahedron[1]);
    Vector3d tetVertex3 = _TV.row(tetrahedron[2]);
    Vector3d tetVertex4 = _TV.row(tetrahedron[3]);
    return (tetVertex1 + tetVertex2 + tetVertex3 + tetVertex4) / 4.f;
}

//
void MeshOperations::partitionVolume(const std::vector<std::unordered_set<int>> &printable_components,
                                     const std::vector<Eigen::Vector3f> &printing_directions,
                                     std::vector<std::vector<Eigen::Vector4i>> &printable_volumes) {
    printable_volumes.clear();
    for (int i = 0; i < printable_components.size(); i++) {
        printable_volumes.push_back({});
    }

    std::vector<Eigen::Vector3f> directions(_num_random_dir_samples);
    sampleRandomDirections(directions);

    // NOTE: make a global variable since this is also used for the visualizer?
    std::unordered_map<int, int> faceToGroup;
    for (int i = 0; i < printable_components.size(); i++) {
        std::unordered_set<int> group_i = printable_components[i];
        for (auto j = group_i.begin(); j != group_i.end(); j++) {
            int face_index = *j;
            assert(!faceToGroup.contains(face_index));
            faceToGroup[face_index] = i;
        }
    }

    for (int i = 0; i < _TT.rows(); i++) {
        Eigen::Vector4i tetrahedron = _TT.row(i).cast<int>();
        Eigen::Vector3f centroid = computeTetCentroid(tetrahedron).cast<float>();

        float minDistance = std::numeric_limits<float>::max();
        int boundaryFace = -1;
        for (Eigen::Vector3f& direction : directions) {
            float distance;
            int intersectedFace = getIntersectionWithDistance(centroid, direction, distance);
            if (intersectedFace >= 0 && distance < minDistance) {
                assert(distance > 0);
                boundaryFace = intersectedFace;
            } else {
                std::cerr << "DID NOT HIT A FACE ON THE BOUNDARY?" << std::endl;
            }
        }

        if (boundaryFace == -1) {
            std::cerr << "something went very wrong here" << std::endl;
        }

        int groupNum = faceToGroup[boundaryFace];
        printable_volumes[groupNum].push_back(tetrahedron);
    }
}

void MeshOperations::updateFaceMap(std::unordered_map<Vector3i, Vector4i, Vector3iHash, Vector3iEqual>& faceMap, Vector3i face, Vector4i tet) {
    if (faceMap.contains(face)) {
        faceMap.erase(face);
    } else {
        faceMap.insert({face, tet});
    }
}

Vector3i MeshOperations::orderVertices(Vector3i& face, Vector4i& tet) {
    // checking of v0, v1, v2 is in CCW order
    Vector3f v0Coords = _vertices[face[0]];
    Vector3f v1Coords = _vertices[face[1]];
    Vector3f v2Coords = _vertices[face[2]];
    Vector3f normal = ((v1Coords - v0Coords).cross(v2Coords - v0Coords)).normalized();
    float d = -1 * (v0Coords).dot(normal);

    Vector3f tetVertex1 = _vertices[tet[0]];
    Vector3f tetVertex2 = _vertices[tet[1]];
    Vector3f tetVertex3 = _vertices[tet[2]];
    Vector3f tetVertex4 = _vertices[tet[3]];
    Vector3f tetCenter = (tetVertex1 + tetVertex2 + tetVertex3 + tetVertex4) / 4.f;
    Vector4f u = {tetCenter[0], tetCenter[1], tetCenter[2], 1}; // u should be centroid of tetrahedron

    Vector4f v{normal[0], normal[1], normal[2], d};
    if (u.dot(v) < 0) {
        return {face[0], face[1], face[2]};
    } else {
        // otherwise this should be CCW order
        v0Coords = _vertices[face[1]];
        v1Coords = _vertices[face[0]];
        v2Coords = _vertices[face[2]];
        normal = ((v1Coords - v0Coords).cross(v2Coords - v0Coords)).normalized();
        d = -1 * (v0Coords).dot(normal);
        v = {normal[0], normal[1], normal[2], d};
        assert(u.dot(v) < 0);
        return {face[1], face[0], face[2]};
    }
}

void MeshOperations::extractSurface(const std::vector<Eigen::Vector4i> &volume, std::vector<Eigen::Vector3i> &surface_faces) {
    // for each tetrahedron
    // get the four faces
    // determine whether the face has already appeared
    std::unordered_map<Vector3i, Vector4i, Vector3iHash, Vector3iEqual> faceMap;
    for (int i = 0; i < volume.size(); i++) {
        Vector3i f0{volume[i][1], volume[i][2], volume[i][3]};
        Vector3i f1{volume[i][0], volume[i][2], volume[i][3]};
        Vector3i f2{volume[i][0], volume[i][1], volume[i][3]};
        Vector3i f3{volume[i][0], volume[i][1], volume[i][2]};
        updateFaceMap(faceMap, f0, volume[i]);
        updateFaceMap(faceMap, f1, volume[i]);
        updateFaceMap(faceMap, f2, volume[i]);
        updateFaceMap(faceMap, f3, volume[i]);
    }

    surface_faces.clear();
    for (auto it = faceMap.begin(); it != faceMap.end(); ++it) {
        Vector3i f = it->first;
        Vector4i tet = it->second;
        Vector3i ordered = orderVertices(f, tet);
        surface_faces.push_back(ordered);
    }
}



