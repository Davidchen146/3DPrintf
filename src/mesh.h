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
    Eigen::Vector3f normal;
    Eigen::Matrix4f Q;
};

struct Face {
    int index;
    Halfedge* halfedge;
    Eigen::Vector3f normal;
    std::vector<Face *> neighbors;
    double area;
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
    std::pair<int, int> vertices;
    Eigen::Vector3f normal;
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

// Really jank code that lets you hash pairs of integers
struct PairHash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &v) const
    {
        return std::hash<T1>()(v.first) ^ std::hash<T2>()(v.second) << 1;
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
    std::pair<int, int> getSortedPair(int x, int y);
    Halfedge* checkHalfEdges(Vertex* source, Vertex* destination);
    void makeHalfEdges(Vertex* vertex1, Vertex* vertex2, Face* faceStruct);
    void convert();
    void validate();

    void flip(Edge* edge);
    Vertex* collapse(Edge* edge, Eigen::Vector4f point);
    Vertex* split(Edge* edge, std::unordered_set<Edge*>* newEdges);

    void simplify(int numFacesToRemove);
    void loopSubdivide();
    void remesh(float w);

    bool removeSpecificEdgeFromEdgeQueue(Edge* e);

    void testFlip(int n);
    void testSplit(int n);
    void testCollapse(int n);

    void updateEdgeCost(Edge* e);

    void testAmbientOcclusion();

    // Accessors for collections of elements
    std::vector<Eigen::Vector3f> getVertices();
    std::vector<Eigen::Vector3i> getFaces();
    const std::unordered_map<int, Vertex*>& getVertexMap();
    const std::unordered_map<int, Face*>& getFaceMap();
    const std::unordered_map<std::pair<int, int>, Edge*, PairHash>& getEdgeMap();

    // Accessors for individual elements
    const Vertex* getVertex(int vertex);
    const Face* getFace(int face);
    const Edge* getEdge(std::pair<int, int> edge);

private:
    int globalVertexIndex;
    std::unordered_map<int, Vertex*>                              _vertexMap;
    std::unordered_set<Halfedge*>                                 _halfEdgeSet;
    std::unordered_map<int, Face*>                                _faceMap;
    std::unordered_map<std::pair<int, int>, Edge*, PairHash>      _edgeMap;
    std::multiset<Edge*, edgeCostComparator>                      _edgeQueue;

    std::vector<Eigen::Vector3f>       _vertices;
    std::vector<Eigen::Vector3i>       _faces;
};

