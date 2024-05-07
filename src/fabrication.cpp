#include "qmath.h"
#include "src/meshoperations.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <QString>
#include <igl/invert_diag.h>
#include <igl/adjacency_matrix.h>

#include <igl/per_vertex_attribute_smoothing.h>
#include <igl/adjacency_list.h>


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
    VectorXd area;
    igl::doublearea(_V.cast<double>(),_F,area);
    double area_avg = area.mean();
    double edge_length = sqrt(4 * area_avg / sqrt(3));
    double volume = pow(edge_length, 3) / (6 * sqrt(2));
    volume /= 2;
    std::cout << "volume: " << volume << std::endl;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5) << volume;
    std::string volume_string = oss.str();
    std::string options = "pq1.414Ya";
    options.append(volume_string);
    options.append("n");
    std::cout << "options: " << options << std::endl;
    igl::copyleft::tetgen::tetrahedralize(_V.cast<double>(), _F, options, _TV, _TT, _TF);

    // Next: let's apply Laplacian Smoothing to only the inside vertices
    std::set<int> newVertexIndices; // to newly created inside-vertices
    // // Step 1: ID all new vertices added from tetrahedralization, subtract
    Eigen::MatrixXd _VCast = _V.cast<double>();
    for (int i = 0; i < _TV.rows(); i++) {
        bool isNew = true;
        for (int j = 0; j < _VCast.rows(); j++) {
            if ((_TV.row(i) - _VCast.row(j)).norm() < 1e-7) { // in case there's some margin of error
                isNew = false;
                break;
            }
        }
        if (isNew) {
            newVertexIndices.insert(i);
        }
    }






    // now we have a set of indices that key into the rows of _TV that ID new vertices.
    // for (int iteration = 0; iteration < 100; iteration++)
    // {
    //     //std::cout << "working" << std::endl;
    //     //Laplacian smoothing -- from
    //     Eigen::SparseMatrix<double> L;
    //     igl::cotmatrix(_TV, _TT, L);

    //     // get mass matrix
    //     Eigen::SparseMatrix<double> M;
    //     igl::massmatrix(_TV, _TT, igl::MASSMATRIX_TYPE_DEFAULT, M);

    //     // invert the mass matrix
    //     Eigen::SparseMatrix<double> M_inv;
    //     igl::invert_diag(M, M_inv);
    //     Eigen::MatrixXd S = M_inv * (L * _TV);

    //     // apply smoothing only to new vertices

    //     for (int idx : newVertexIndices) {
    //         //std::cout << S.row(idx) << std::endl;
    //         _TV.row(idx) = _TV.row(idx) + 100000000000000.0 * S.row(idx); // adjust multiplier -- control the amount of smoothing
    //     }
    //     // Eigen::MatrixXd smoothedTV;

    //     // igl::per_vertex_attribute_smoothing(_TV, _TF, smoothedTV);

    //     // for (int idx : newVertexIndices) {
    //     //     _TV.row(idx) = smoothedTV.row(idx);
    //     // }
    // }
}

Eigen::Vector3d MeshOperations::computeTetCentroid(Eigen::Vector4i &tetrahedron) {
    Vector3d tetVertex1 = _TV.row(tetrahedron[0]);
    Vector3d tetVertex2 = _TV.row(tetrahedron[1]);
    Vector3d tetVertex3 = _TV.row(tetrahedron[2]);
    Vector3d tetVertex4 = _TV.row(tetrahedron[3]);
    return (tetVertex1 + tetVertex2 + tetVertex3 + tetVertex4) / 4.f;
}

void MeshOperations::getComponentBoundingBox(const std::unordered_set<int> &component, Eigen::Vector3d& min, Eigen::Vector3d& max) {
    std::unordered_set<int> vertexIndices;
    for (int f : component) {
        Vector3i face = _faces[f];
        if (!vertexIndices.contains(face[0])) {
            vertexIndices.insert(face[0]);
        }
        if (!vertexIndices.contains(face[1])) {
            vertexIndices.insert(face[1]);
        }
        if (!vertexIndices.contains(face[2])) {
            vertexIndices.insert(face[2]);
        }
    }
    MatrixXf V;
    V.resize(vertexIndices.size(), 3);
    int num = 0;
    for (int vertex : vertexIndices) {
        V.row(num) = _vertices[vertex];
        num++;
    }
    // get the bounding box of printable component?
    min = _V.cast<double>().colwise().minCoeff();
    max = _V.cast<double>().colwise().maxCoeff();
}

void MeshOperations::partitionVolume(const std::vector<std::unordered_set<int>> &printable_components,
                                     std::vector<std::vector<Eigen::Vector4i>> &printable_volumes) {
    printable_volumes.clear();
    for (int i = 0; i < printable_components.size(); i++) {
        printable_volumes.push_back({});
    }

    std::vector<Eigen::Vector3d> boxMin;
    std::vector<Eigen::Vector3d> boxMax;
    boxMin.resize(printable_components.size());
    boxMax.resize(printable_components.size());
    for (int i = 0; i < printable_components.size(); i++) {
        Eigen::Vector3d min;
        Eigen::Vector3d max;
        getComponentBoundingBox(printable_components[i], min, max);
        boxMin[i] = min;
        boxMax[i] = max;
    }

    int n = 512;
    std::vector<Eigen::Vector3f> directions(n);
    float phi = M_PI * (sqrt(5) - 1);
    for (int i = 0; i < n; i++) {
        float y = 1 - ((float) i / (n - 1)) * 2;
        float radius = sqrt(1 - y * y);
        float theta = phi * i;
        float x = cos(theta) * radius;
        float z = sin(theta) * radius;
        directions[i] = Vector3f(x, y, z);
    }

    if (faceToGroup.size() == 0) {
        std::cerr << "MUST CALL visualize() first to initialize faceToGroup" << std::endl;
        return;
    }

    for (int i = 0; i < _TT.rows(); i++) {
        Eigen::Vector4i tetrahedron = _TT.row(i).cast<int>();
        Eigen::Vector3f centroid = computeTetCentroid(tetrahedron).cast<float>();

        float minDistance = std::numeric_limits<float>::max();
        int boundaryFace = -1;
        for (Eigen::Vector3f& direction : directions) {
            float distance;
            int intersectedFace = getIntersectionWithDistance(centroid, direction, distance);
            if (intersectedFace >= 0 && distance < minDistance && distance > 0) {
                int groupNum = faceToGroup[intersectedFace];
                Eigen::Vector3d min = boxMin[groupNum];
                Eigen::Vector3d max = boxMax[groupNum];
                if (centroid[0] >= min[0] && centroid[0] <= max[0] && centroid[1] >= min[1] && centroid[1] <= max[1] && centroid[2] >= min[2] && centroid[2] <= max[2]) {
                    minDistance = distance;
                    boundaryFace = intersectedFace;
                }
            }
            if (intersectedFace == -1) {
                //std::cerr << "DID NOT HIT A FACE ON THE BOUNDARY?" << std::endl;
            }
        }
        int groupNum = faceToGroup[boundaryFace];
        printable_volumes[groupNum].push_back(tetrahedron);
    }

    // ALTERNATIVE APPROACH
    // need igl::adjacency_list which constructs the graph adjacency list of a given mesh (V, F)
    // gives you a vector<vector<int>> containing at row i the adjacent vertices of vertex i
    // but if i have the adjacent vertices

}

// Propagate a single volume collection one step into the mesh
void MeshOperations::propagateVolume(PrintableVolume* volume,
                     std::unordered_map<Eigen::Vector3i, std::pair<int, int>, Vector3iHash, Vector3iEqual> &faceToTet,
                     std::unordered_set<int> &unassignedTets) {

    unordered_set<int> nextLayer;
    unordered_set<int> currLayer = volume->currentLayer;
    for (int t: currLayer) {
        unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> faces;
        getTetFaces(t, faces);

        for (auto it = faces.begin(); it != faces.end(); ++it) {
            Vector3i face = *it;
            pair<int, int> adjacentTets = faceToTet[face];
            int targetTetIndex;
            // Get the tet's neighbor
            if (adjacentTets.first != t) {
                targetTetIndex = adjacentTets.first;
            } else {
                targetTetIndex = adjacentTets.second;
            }
            // If the neighboring tet is unassigned
            if (unassignedTets.contains(targetTetIndex) && !nextLayer.contains(targetTetIndex)) {
                unassignedTets.erase(targetTetIndex);
                volume->totalVolume.insert(targetTetIndex);
                nextLayer.insert(targetTetIndex);
                // Don't consider any other faces. We only want to propagate each tet once
                continue;
            }
        }
    }
    volume->currentLayer = nextLayer;
}

void MeshOperations::getTetFaces(int tetrahedron, std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> &faces) {
    Eigen::Vector4i tet = _TT.row(tetrahedron).cast<int>();
    Eigen::Vector3i face1 = {tet[0], tet[1], tet[2]};
    Eigen::Vector3i face2 = {tet[0], tet[1], tet[3]};
    Eigen::Vector3i face3 = {tet[0], tet[2], tet[3]};
    Eigen::Vector3i face4 = {tet[1], tet[2], tet[3]};
    // ASSUMING Vector3iEqual IS GONNA SORT EVERYTHING IN CONSISTENT ORDER
    faces.insert(face1);
    faces.insert(face2);
    faces.insert(face3);
    faces.insert(face4);
}

void MeshOperations::initializePrintableVolumes(const std::vector<std::unordered_set<int>> &printable_components,
                                std::vector<PrintableVolume*> &volumes_to_propagate,
                                std::unordered_map<Eigen::Vector3i, std::pair<int, int>, Vector3iHash, Vector3iEqual> &faceToTet,
                                std::unordered_set<int> &unassignedTets) {

    for (int i = 0; i < printable_components.size(); i++) {
        PrintableVolume* volume = new PrintableVolume;
        for (int f: printable_components[i]) {
            Vector3i face = _faces[f];
            pair<int, int> adjacentTets = faceToTet[face];
            // These are surface faces, so one of the ints in pair should be -1
            int surfaceTet;
            if (adjacentTets.first == -1) {
                surfaceTet = adjacentTets.second;
            } else {
                surfaceTet = adjacentTets.first;
            }

            volume->totalVolume.insert(surfaceTet);
            volume->currentLayer.insert(surfaceTet);
            unassignedTets.erase(surfaceTet);
        }
        volumes_to_propagate.push_back(volume);
    }
}

void MeshOperations::getFacesFromTet(const std::vector<Eigen::Vector4i>& volume, Eigen::MatrixXi& faces, std::unordered_set<Vector3i, Vector3iHash, Vector3iEqual>& exposedFaces) {
    std::unordered_map<Vector3i, int, Vector3iHash, Vector3iEqual> faceMap;
    // (theoretically Vector3iEqual will sort the vector) --> so duplicate faces with different orderings won't exist in the set
    for (int i = 0; i < volume.size(); i++) {
        Eigen::Vector4i tet = volume[i];
        Eigen::Vector3i face1 = {tet[0], tet[1], tet[2]};
        Eigen::Vector3i face2 = {tet[0], tet[1], tet[3]};
        Eigen::Vector3i face3 = {tet[0], tet[2], tet[3]};
        Eigen::Vector3i face4 = {tet[1], tet[2], tet[3]};
        if (faceMap.contains(face1)) {
            faceMap[face1] += 1;
        } else {
            faceMap[face1] = 1;
        }
        if (faceMap.contains(face2)) {
            faceMap[face2] += 1;
        } else {
            faceMap[face2] = 1;
        }
        if (faceMap.contains(face3)) {
            faceMap[face3] += 1;
        } else {
            faceMap[face3] = 1;
        }
        if (faceMap.contains(face4)) {
            faceMap[face4] += 1;
        } else {
            faceMap[face4] = 1;
        }
    }
    faces.resize(faceMap.size(), 3);
    int rowNum = 0;
    exposedFaces.clear();
    for (const std::pair<Vector3i, int>& pair : faceMap) {
        Vector3i face = pair.first;
        faces.row(rowNum) = face;
        int count = pair.second;
        if (count == 1) {
            exposedFaces.insert(face);
        }
        rowNum++;
    }
}

void MeshOperations::pruneVolume(std::vector<std::vector<Eigen::Vector4i>> &printable_volumes) {
    // each Eigen::Vector4i represents a tetrahedron --> each int is an index into TV
    for (int i = 0; i < printable_volumes.size(); i++) {
        MatrixXi faces;
        std::unordered_set<Vector3i, Vector3iHash, Vector3iEqual> exposedFaces;
        getFacesFromTet(printable_volumes[i], faces, exposedFaces);
        vector<vector<double>> A; // containing at row i the adjacent vertices of vertex i
        // from the printable volumes get the faces
        igl::adjacency_list(faces, A);
        std::vector<Eigen::Vector4i> volume = printable_volumes[i];
        // for (Eigen::Vector4i& tetrahedron : volume) {
        std::vector<int> toErase;
        for (int j = 0; j < volume.size(); j++) {
            Eigen::Vector4i tetrahedron = volume[j];
            bool v1_dangling = A[tetrahedron[0]].size() == 3;
            bool v2_dangling = A[tetrahedron[1]].size() == 3;
            bool v3_dangling = A[tetrahedron[2]].size() == 3;
            bool v4_dangling = A[tetrahedron[3]].size() == 3;
            int total_true = v1_dangling + v2_dangling + v3_dangling + v4_dangling;
            if (total_true >= 2) {
                // remove the tetrahedron from the printable volume
                toErase.push_back(j);
            } else {
                Eigen::Vector3i face1 = {tetrahedron[0], tetrahedron[1], tetrahedron[2]};
                Eigen::Vector3i face2 = {tetrahedron[0], tetrahedron[1], tetrahedron[3]};
                Eigen::Vector3i face3 = {tetrahedron[0], tetrahedron[2], tetrahedron[3]};
                Eigen::Vector3i face4 = {tetrahedron[1], tetrahedron[2], tetrahedron[3]};
                bool f1_exposed = exposedFaces.contains(face1);
                bool f2_exposed = exposedFaces.contains(face2);
                bool f3_exposed = exposedFaces.contains(face3);
                bool f4_exposed = exposedFaces.contains(face4);
                int num_exposed = f1_exposed + f2_exposed + f3_exposed + f4_exposed;
                if (num_exposed == 4) {
                    toErase.push_back(j);
                }
            }
        }
        if (toErase.size() > 0) {
            std::cout << "size: " << toErase.size() << std::endl;
            std::sort(toErase.begin(), toErase.end(), std::greater<int>());
            for (int j = 0; j < toErase.size(); j++) {
                auto it = printable_volumes[i].begin() + toErase[j];
                printable_volumes[i].erase(it);
            }
        }
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
    Vector3d v0Coords = _TV.row(face[0]);
    Vector3d v1Coords = _TV.row(face[1]);
    Vector3d v2Coords = _TV.row(face[2]);
    Vector3d normal = ((v1Coords - v0Coords).cross(v2Coords - v0Coords)).normalized();
    double d = -1 * (v0Coords).dot(normal);

    Vector3d tetVertex1 = _TV.row(tet[0]);
    Vector3d tetVertex2 = _TV.row(tet[1]);
    Vector3d tetVertex3 = _TV.row(tet[2]);
    Vector3d tetVertex4 = _TV.row(tet[3]);
    Vector3d tetCenter = (tetVertex1 + tetVertex2 + tetVertex3 + tetVertex4) / 4.f;
    Vector4d u = {tetCenter[0], tetCenter[1], tetCenter[2], 1}; // u should be centroid of tetrahedron

    Vector4d v{normal[0], normal[1], normal[2], d};
    if (u.dot(v) < 0) {
        return {face[0], face[1], face[2]};
    } else {
        // otherwise this should be CCW order
        v0Coords = _TV.row(face[1]);
        v1Coords = _TV.row(face[0]);
        v2Coords = _TV.row(face[2]);
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
    std::unordered_map<Vector3i, Vector4i, Vector3iHash, Vector3iEqual> surfaceFaceMap;
    for (int i = 0; i < volume.size(); i++) {
        Vector3i f0{volume[i][1], volume[i][2], volume[i][3]};
        Vector3i f1{volume[i][0], volume[i][2], volume[i][3]};
        Vector3i f2{volume[i][0], volume[i][1], volume[i][3]};
        Vector3i f3{volume[i][0], volume[i][1], volume[i][2]};
        updateFaceMap(surfaceFaceMap, f0, volume[i]);
        updateFaceMap(surfaceFaceMap, f1, volume[i]);
        updateFaceMap(surfaceFaceMap, f2, volume[i]);
        updateFaceMap(surfaceFaceMap, f3, volume[i]);
    }

    surface_faces.clear();
    for (auto it = surfaceFaceMap.begin(); it != surfaceFaceMap.end(); ++it) {
        Vector3i f = it->first;
        Vector4i tet = it->second;
        Vector3i ordered = orderVertices(f, tet);
        surface_faces.push_back(ordered);
    }
}



