#pragma once

#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>

#include "igl/embree/ambient_occlusion.h"
#include <igl/per_vertex_normals.h>

#include "Eigen/StdVector"
#include "Eigen/Dense"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i);

struct Halfedge;
struct Vertex;
struct Face;
struct Edge;


struct Vertex {
    Halfedge* halfedge;
    int index;
    Eigen::Vector3f p; // p is the coordinates
    Eigen::Matrix4f Q;
};

struct Face {
    int index;
    Halfedge* halfedge;
    Eigen::Vector3f normal;
    std::vector<Face *> neighbors;
};

struct Halfedge {
    Halfedge* twin;
    Halfedge* next;
    Vertex* source;
    Vertex* destination;
    Face* face;
    Edge* edge;
};

struct Edge {
    Halfedge* halfedge;
    Eigen::Vector4f minCostPoint;
    float error;
};

struct edgeCostComparator {
    bool operator() (const Edge* e1, const Edge* e2) const {
        return e1->error < e2->error;
    }
};


struct Vector3fCompare {
    bool operator()(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) const {
        if (v1.x() < v2.x()) return true;
        if (v1.x() > v2.x()) return false;
        if (v1.y() < v2.y()) return true;
        if (v1.y() > v2.y()) return false;
        return v1.z() < v2.z();
    }
};

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
                         const std::vector<Eigen::Vector3i> &faces);

    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);
    void preProcess();
    Halfedge* checkHalfEdges(Vertex* source, Vertex* destination);
    void makeHalfEdges(Vertex* vertex1, Vertex* vertex2, Face* faceStruct);
    void convert();
    void validate();

    void flip(Edge* edge);
    Vertex* collapse(Edge* edge, Eigen::Vector4f point);

    void simplify(int numFacesToRemove);

    bool removeSpecificEdgeFromEdgeQueue(Edge* e);

    void testFlip(int n);
    void testSplit(int n);
    void testCollapse(int n);

    void updateEdgeCost(Edge* e);

    void testAmbientOcclusion();

    std::vector<Eigen::Vector3f> getVertices();
    std::vector<Eigen::Vector3i> getFaces();
    std::unordered_set<Face *> getFaceSet();

private:
    int globalVertexIndex;
    std::unordered_map<int, Vertex*>                              _vertexMap;
    std::unordered_set<Halfedge*>                                 _halfEdgeSet;
    std::unordered_set<Face*>                                     _faceSet;
    std::unordered_set<Edge*>                                     _edgeSet;
    std::multiset<Edge*, edgeCostComparator>                      _edgeQueue;

    std::vector<Eigen::Vector3f>       _vertices;
    std::vector<Eigen::Vector3i>       _faces;
};

