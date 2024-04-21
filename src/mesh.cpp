#include "src/mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const vector<Vector3f> &vertices,
                           const vector<Vector3i> &faces)
{
    // Copy vertices and faces into internal vector
    _vertices = vertices;
    _faces    = faces;
}

Halfedge* Mesh::checkHalfEdges(Vertex* source, Vertex* destination) {
    for (Halfedge* halfedge: _halfEdgeSet) {
        if (halfedge->source == source && halfedge->destination == destination) {
            return halfedge;
        }
    }
    return nullptr;
}

void Mesh::makeHalfEdges(Vertex* vertex1, Vertex* vertex2, Face* faceStruct) {
    Halfedge* existingHalfEdge = checkHalfEdges(vertex1, vertex2);
    if (existingHalfEdge == nullptr) {
        // If half edge twins don't exist, make them
        Halfedge* halfedge = new Halfedge{nullptr, nullptr, vertex1, vertex2, faceStruct, nullptr};
        Halfedge* twin = new Halfedge{halfedge, nullptr, vertex2, vertex1, nullptr, nullptr};
        Edge* edge = new Edge{halfedge, std::make_pair(vertex1->index, vertex2->index)};
        if (vertex2->index < vertex1->index) {
            edge->vertices = std::make_pair(vertex2->index, vertex1->index);
        }
        // Set the first half edge's twin
        halfedge->twin = twin;
        // Set the vertex's half edge
        vertex1->halfedge = halfedge;
        // Set the edge that each half-edge belongs to
        halfedge->edge = edge;
        twin->edge = edge;
        // Add to set of half edges
        _halfEdgeSet.insert(halfedge);
        _halfEdgeSet.insert(twin);
        // Add to set of edges
        _edgeMap[edge->vertices] = edge;
        faceStruct->halfedge = vertex1->halfedge;
    } else {
        existingHalfEdge->face = faceStruct;
        faceStruct->halfedge = existingHalfEdge;
        vertex1->halfedge = existingHalfEdge;
    }
}

Eigen::Vector3f calculateSurfaceNormal(const Eigen::Vector3f v1, const Eigen::Vector3f v2, const Eigen::Vector3f v3) {
    // Compute edge vectors
    Eigen::Vector3f e1 = v2 - v1;
    Eigen::Vector3f e2 = v3 - v1;

    // Compute surface normal
    Eigen::Vector3f normal = e1.cross(e2);

    // Normalize the surface normal
    normal.normalize();

    return normal;
}

void Mesh::preProcess() {
    for (int i = 0; i < _vertices.size(); i++) {
        Vector3f originalVertex = _vertices[i];
        Vertex* v = new Vertex{nullptr, i, originalVertex};
        // Populate vertex map, with position as the key and Vertex struct as value
        _vertexMap.insert({i, v});
    }
    globalVertexIndex = _vertexMap.size();

    for (int i = 0; i < _faces.size(); i++) {
        // Loop through each face, construct pairs of half edges if it does not already exist
        Vector3i face = _faces[i];

        // Each vertex on a face is stored as indices in the _vertices array
        Vector3f v1 = _vertices[face[0]];
        Vector3f v2 = _vertices[face[1]];
        Vector3f v3 = _vertices[face[2]];

        // Fill in later
        Face* faceStruct = new Face{i, nullptr, calculateSurfaceNormal(v1, v2, v3), {}};

        // Get our vertex structs
        Vertex* vertex1 = _vertexMap[face[0]];
        Vertex* vertex2 = _vertexMap[face[1]];
        Vertex* vertex3 = _vertexMap[face[2]];

        // We make existing half edges counter-clockwise, ie. 1-2, 2-3, 3-1
        makeHalfEdges(vertex1, vertex2, faceStruct);
        makeHalfEdges(vertex2, vertex3, faceStruct);
        makeHalfEdges(vertex3, vertex1, faceStruct);

        // Glue all internal half edges together
        vertex1->halfedge->next = vertex2->halfedge;
        vertex2->halfedge->next = vertex3->halfedge;
        vertex3->halfedge->next = vertex1->halfedge;

        // Add face to map
        assert(!_faceMap.contains(i));
        _faceMap[i] = faceStruct;
    }

    // NOTE: consider whether these normals should be area weighted
    // Imma go with "No"

    // compute vertex normals (by averaging the normals of all neighboring faces)
    for (const auto& pair : _vertexMap) {
        Vertex *v = pair.second;
        Halfedge *h = v->halfedge;
        Vector3f vertexNormal{0, 0, 0};
        int numNeighbors = 0;
        do {
            vertexNormal += h->face->normal;
            numNeighbors++;
        } while (h != v->halfedge);
        vertexNormal /= numNeighbors;
        v->normal = vertexNormal;
    }

    // edge normal is just average of two adjacent faces
    for (const auto& pair : _edgeMap) {
        Edge *e = pair.second;
        Vector3f edgeNormal = e->halfedge->face->normal + e->halfedge->twin->face->normal;
        edgeNormal /= 2;
        e->normal = edgeNormal;
    }

    // get neighboring faces and edges for each face (to store in face struct)
    for (const auto& pair : _faceMap) {
        Face *f_i = pair.second;
        Halfedge *h = f_i->halfedge;
        vector<Face *> neighbors(3);
        vector<Edge *> edges(3);
        int neighbor_num = 0;
        do {
            Face *f_j = h->twin->face;
            neighbors[neighbor_num] = f_j;
            Edge *e = h->edge;
            edges[neighbor_num] = e;
            neighbor_num++;
            h = h->next;
        } while (h != f_i->halfedge);
        assert(neighbor_num == 3);
        f_i->neighbors = neighbors;
        f_i->edges = edges;
    }   
}

// Converts our data structure back into the original format of _vertices and _faces
void Mesh::convert() {
    vector<Vector3f> finalVerticesList;
    vector<Vector3i> finalFacesList;

    // Loop through our vertices, update Vertex struct's index, and append position to final vertices list
    int index = 0;
    for (auto const& [key, val] : _vertexMap) {
        val->index = index;
        finalVerticesList.push_back(val->p);
        index++;
    }

    // Loop through faces, record indices of each vertex and append to final faces list
    for (const auto& pair : _faceMap) {
        Face *face = pair.second;
        vector<int> indices;
        Halfedge* h = face->halfedge;

        do {
            indices.push_back(h->source->index);
            h = h->next;
        }
        while (h != face->halfedge);

        Vector3i finalIndices(indices[0], indices[1], indices[2]);
        finalFacesList.push_back(finalIndices);
    }

    _vertices = finalVerticesList;
    _faces = finalFacesList;
}

void Mesh::loadFromFile(const string &filePath)
{
    tinyobj::attrib_t attrib;
    vector<tinyobj::shape_t> shapes;
    vector<tinyobj::material_t> materials;

    QFileInfo info(QString(filePath.c_str()));
    string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if (!err.empty()) {
        cerr << err << endl;
    }

    if (!ret) {
        cerr << "Failed to load/parse .obj file " << filePath << endl;
        return;
    }

    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                face[v] = idx.vertex_index;
            }
            _faces.push_back(face);

            index_offset += fv;
        }
    }
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        _vertices.emplace_back(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    }
    cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << endl;
}

void Mesh::saveToFile(const string &filePath)
{
    ofstream outfile;
    outfile.open(filePath);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        const Vector3f &v = _vertices[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << endl;
    }

    outfile.close();
}

int countDegree(Vertex* v) {
    int degree = 0;
    Halfedge* h = v->halfedge;

    do {
        degree++;
        h = h->next->next->twin;
    }
    while (h != v->halfedge);

    return degree;
}

void Mesh::flip(Edge* edge){
    Halfedge* h = edge->halfedge;
    Vertex* source = h->source;
    Vertex* destination = h->destination;

    if (countDegree(source) == 3 || countDegree(destination) == 3) {
        return;
    }
    // Define vertices a,b,c,d, let current connection be (c,b)
    // Delete (c,b) and twin (b,c), create (a,d) and twin (d,a)
    // Delete faces (a,b,c) and (c,b,d), create new ones (a,d,c) and (a,b,d)

    Halfedge* cb = h;
    Halfedge* bd = cb->next;
    Halfedge* dc = bd->next;

    Halfedge* bc = cb->twin;
    Halfedge* ca = bc->next;
    Halfedge* ab = ca->next;

    Face* face = cb->face;
    Face* twinFace = cb->twin->face;
    // Reassign face half edges since we are deleting the current half edge
    face->halfedge = dc;
    twinFace->halfedge = ab;

    Vertex* c = source;
    Vertex* b = destination;
    Vertex* d = cb->next->destination;
    Vertex* a = cb->twin->next->destination;

    // Fix vertex half edge pointers since we are deleting cb
    c->halfedge = ca;
    b->halfedge = bd;

    // Create new connections (a,d) and (d,a)
    Halfedge* ad = new Halfedge{nullptr, nullptr, a, d, dc->face, nullptr};
    Halfedge* da = new Halfedge{ad, nullptr, d, a, ab->face, nullptr};
    ad->twin = da;
    a->halfedge = ad;
    d->halfedge = da;

    // Create new edge
    Edge* ad_edge = new Edge{ad, std::make_pair(a->index, d->index)};
    if (d->index < a->index) {
        ad_edge->vertices = std::make_pair(d->index, a->index);
    }
    ad->edge = ad_edge;
    da->edge = ad_edge;

    // Add half edges to set
    _halfEdgeSet.insert(ad);
    _halfEdgeSet.insert(da);

    // Add edge to set
    assert(_edgeMap.contains(std::make_pair(a->index, d->index)));
    _edgeMap[ad_edge->vertices] = ad_edge;

    // Fix faces for (c,a) and (b,d)
    ca->face = dc->face;
    bd->face = ab->face;

    // Glue faces together
    dc->next = ca;
    ca->next = ad;
    ad->next = dc;

    ab->next = bd;
    bd->next = da;
    da->next = ab;

    // Remove cb AND bc from our set of half edges and free the memory
    _halfEdgeSet.erase(cb);
    _halfEdgeSet.erase(bc);
    delete cb;
    delete bc;

    delete edge;
}

vector<Vertex*> findNeighbors(Vertex* v) {
    vector<Vertex*> neighbors;

    Halfedge* h = v->halfedge;
    do {
        neighbors.push_back(h->destination);
        h = h->next->next->twin;
    }
    while (h != v->halfedge);

    return neighbors;
}

bool collapseValidate(Vertex* c, Vertex* d) {
    vector<Vertex*> cNeighbors = findNeighbors(c);
    vector<Vertex*> dNeighbors = findNeighbors(d);
    vector<Vertex*> shared;

    std::set<Vertex*> cNeighborsSet(cNeighbors.begin(), cNeighbors.end());
    // Find all vertices that c and d share as neighbors
    for (Vertex* v: dNeighbors) {
        if (cNeighborsSet.contains(v)) {
            shared.push_back(v);
        }
    }

    // If edge endpoints have more than 2 shared neighbors
    if (shared.size() > 2) {
        return false;
    }

    // If edge endpoints have a neighbor with degree 3
    for (Vertex* v: shared) {
        if (countDegree(v) == 3) {
            return false;
        }
    }
    return true;
}

Vertex* Mesh::collapse(Edge* edge, Vector4f point) {
    // Let's call the edge we are collapsing cd, dc
    Halfedge* h = edge->halfedge;
    Halfedge* cd = h;
    Halfedge* dc = cd->twin;
    Face* face = cd->face;
    Face* twinFace = dc->face;

    Vertex* c = cd->source;
    Vertex* d = cd->destination;

    if (!collapseValidate(c, d)){
        return nullptr;
    }

    Vector3f newVertexCoord(point[0], point[1], point[2]);
    // Make and add the new Vertex struct
    Vertex* m = new Vertex{nullptr, globalVertexIndex, newVertexCoord};
    _vertexMap.insert({globalVertexIndex, m});
    globalVertexIndex++;
    m->Q = c->Q + d->Q;

    Halfedge* da = cd->next;
    Halfedge* ad = da->twin;
    Halfedge* ac = da->next;
    Halfedge* ca = ac->twin;

    Halfedge* cb = dc->next;
    Halfedge* bc = cb->twin;
    Halfedge* bd = cb->next;
    Halfedge* db = bd->twin;

    // Reassign vertex halfedges
    Vertex* a = ad->source;
    Vertex* b = bc->source;

    a->halfedge = ad;
    b->halfedge = bc;

    // Change all adjacent edges to appropriate endpoint m
    Halfedge* c_h = cd;
    do {
        c_h->source = m;
        c_h->twin->destination = m;
        c_h = c_h->twin->next;
    }
    while (c_h != cd);

    Halfedge* d_h = dc;
    do {
        d_h->source = m;
        d_h->twin->destination = m;
        d_h = d_h->twin->next;
    }
    while (d_h != dc);

    // Glue half edges together to eliminate faces
    ca->edge->halfedge = ca;
    ca->twin = ad;
    ad->twin = ca;
    // ad gets absorbed into ca's edge
    Edge* ad_edge = ad->edge;
    ad->edge = ca->edge;

    bc->edge->halfedge = bc;
    bc->twin = db;
    db->twin = bc;
    // db gets absorbed into bc's edge
    Edge* db_edge = db->edge;
    db->edge = bc->edge;

    m->halfedge = ca;


    removeSpecificEdgeFromEdgeQueue(edge);
    removeSpecificEdgeFromEdgeQueue(ad_edge);
    removeSpecificEdgeFromEdgeQueue(db_edge);
    // delete all junk values:
    // edges: cd_edge, ad_edge, db_edge
    // half edges: da, ac, cd, dc, cb, bd
    // vertices: c, d
    // faces: cda, cbd
    _halfEdgeSet.erase(da);
    _halfEdgeSet.erase(ac);
    _halfEdgeSet.erase(cd);
    _halfEdgeSet.erase(dc);
    _halfEdgeSet.erase(cb);
    _halfEdgeSet.erase(bd);
    delete da;
    delete ac;
    delete cd;
    delete dc;
    delete cb;
    delete bd;


    _edgeMap.erase(edge->vertices);
    _edgeMap.erase(ad_edge->vertices);
    _edgeMap.erase(db_edge->vertices);
    delete edge;
    delete ad_edge;
    delete db_edge;

    _vertexMap.erase(c->index);
    _vertexMap.erase(d->index);
    delete c;
    delete d;

    _faceMap.erase(face->index);
    _faceMap.erase(twinFace->index);
    delete face;
    delete twinFace;

    return m;
}

Matrix4f getFaceQ(Face* f, Vertex* u) {
    Vector3f v1 = f->halfedge->source->p;
    Vector3f v2 = f->halfedge->next->source->p;
    Vector3f v3 = f->halfedge->next->next->source->p;

    Vector3f N = calculateSurfaceNormal(v1, v2, v3);
    float d = -v1.dot(N);

    Vector4f v(N[0], N[1], N[2], d);

    Matrix4f Q = v*(v.transpose());

    return Q;
}

Matrix4f getVertexQ(Vertex* v) {
    Halfedge* h = v->halfedge;
    Matrix4f Q = Matrix4f::Zero();

    do {
        Q += getFaceQ(h->face, v);
        h = h->twin->next;
    }
    while (h != v->halfedge);

    return Q;
}

bool isMatrixInvertible(Matrix4f* m) {
    return m->determinant() != 0.f;
}

unordered_set<Edge*> findEdgeNeighbors(Vertex* v) {
    unordered_set<Edge*> neighbors;
    Halfedge* h = v->halfedge;

    do {
        neighbors.insert(h->edge);
        h = h->twin->next;
    }
    while (h != v->halfedge);

    return neighbors;
}

void Mesh::updateEdgeCost(Edge* e) {
    Halfedge* h = e->halfedge;
    Vertex* v1 = h->source;
    Vertex* v2 = h->destination;

    // C is a column vector of [0,0,0,1]
    Vector4f c(0, 0, 0, 1);
    // c = c.transpose();

    Matrix4f Q = v1->Q + v2->Q;
    Matrix4f Q_Bar = Q;
    Q_Bar.row(3) << 0, 0, 0, 1;

    Vector4f minCostPoint;

    if (isMatrixInvertible(&Q)) {
        minCostPoint = (Q_Bar.inverse())*c;
    } else {
        // If Q is not invertible, the minimum cost is either the midpoint or one of the endpoints, whichever has less error
        Vector3f midpoint = (v1->p + v2->p) / 2.f;
        Vector4f mp4(midpoint[0], midpoint[1], midpoint[2], 1.f);
        Vector3f ep1 = v1->p;
        Vector4f ep1_4(ep1[0], ep1[1], ep1[2], 1.f);
        Vector3f ep2 = v2->p;
        Vector4f ep2_4(ep2[0], ep2[1], ep2[2], 1.f);

        float errorMP = mp4.transpose()*Q*mp4;
        float errorEP1 = ep1_4.transpose()*Q*ep1_4;
        float errorEP2 = ep2_4.transpose()*Q*ep2_4;

        if (errorMP < errorEP1 && errorMP < errorEP2) {
            minCostPoint = mp4;
        } else if (errorEP1 < errorMP && errorEP1 < errorEP2) {
            minCostPoint = ep1_4;
        } else {
            minCostPoint = ep2_4;
        }
    }

    float error = minCostPoint.transpose()*Q*minCostPoint;
    error = std::max(error, 0.f);
    assert(error >= 0.f);
    e->minCostPoint = minCostPoint;
    e->error = error;
}

void validateEdgeQueue(std::multiset<Edge*, edgeCostComparator>* edgeQueue) {
    for (Edge* edge: *edgeQueue) {
        assert(edge->halfedge != nullptr);
        Halfedge* h = edge->halfedge;
        assert(h->edge == edge && h->twin->edge == edge);
        assert(h == h->twin->twin);
    }
}

bool Mesh::removeSpecificEdgeFromEdgeQueue(Edge* e) {
    if (_edgeQueue.size() == 0) {
        return false;
    }

    auto range = _edgeQueue.equal_range(e);

    if (range.first == range.second) {
        return false;
    }

    for (auto it = range.first; it != range.second; ++it) {
        if (*it == e) {
            _edgeQueue.erase(it);
            return true;
        }
    }
    return false;
}

void Mesh::simplify(int numFacesToRemove){
    int targetNumFaces = _faceMap.size() - numFacesToRemove;
    targetNumFaces = std::max(4, targetNumFaces);

    // Initialize Q matrices
    for (auto const& [index, vertex] : _vertexMap) {
        vertex->Q = getVertexQ(vertex);
    }

    // Initialize minimum cost and points for each edge
    for (const auto& pair : _edgeMap) {
        Edge *e = pair.second;
        updateEdgeCost(e);
        _edgeQueue.insert(e);
    }

    validate();

    while (_faceMap.size() > targetNumFaces) {
        // ** When getting neighbors, beware that some edges may be freed from memory after collapse()
        std::unordered_set<Edge*> neighboringEdges;

        auto firstEdgeIterator = _edgeQueue.begin();
        Edge* edgeToCollapse = *firstEdgeIterator;
        _edgeQueue.erase(firstEdgeIterator);

        if (!_edgeMap.contains(edgeToCollapse->vertices)) {
            // If the edge no longer exists due to collapse
            continue;
        }

        // ** Use iterators instead of Edge pointers when removing elements from queue whenever possible

        Vertex* m = collapse(edgeToCollapse, edgeToCollapse->minCostPoint);
        validate();

        if (m == nullptr) {
            // If the collapse would cause mesh to become non-manifold
            continue;
        }

        // Update cost of neighboring edges by removing them from queue and re-adding them
        neighboringEdges = findEdgeNeighbors(m);
        for (Edge* e: neighboringEdges) {
            removeSpecificEdgeFromEdgeQueue(e);
            updateEdgeCost(e);
            _edgeQueue.insert(e);
        }

    }

}

Vertex* Mesh::split(Edge* edge, unordered_set<Edge*>* newEdges) {
    Halfedge* h = edge->halfedge;

    Halfedge* cb = h;
    Halfedge* bd = cb->next;
    Halfedge* dc = bd->next;

    Halfedge* bc = cb->twin;
    Halfedge* ca = bc->next;
    Halfedge* ab = ca->next;

    Vertex* c = cb->source;
    Vertex* b = cb->destination;
    Vertex* d = bd->destination;
    Vertex* a = ca->destination;

    Face* face1 = cb->face;
    Face* face2 = bc->face;

    // Make new vertex
    Vector3f newVertexCoord = (c->p + b->p) / 2.f;
    Vertex* m = new Vertex{nullptr, globalVertexIndex, newVertexCoord};
    assert(!_vertexMap.contains(globalVertexIndex));
    _vertexMap.insert({globalVertexIndex, m});
    globalVertexIndex++;

    // Make new vertices and faces
    // NOTE: i don't see how we would ever need to split edges but if we do we need to give these new faces indices
    int numFaces = _faceMap.size();
    Face* face3 = new Face{numFaces};
    Face* face4 = new Face{numFaces + 1};
    assert(!_faceMap.contains(numFaces));
    assert(!_faceMap.contains(numFaces + 1));
    _faceMap[numFaces] = face3;

    // Make half edges
    Halfedge* cm = new Halfedge{nullptr, nullptr, c, m, face1, nullptr};
    Halfedge* md = new Halfedge{nullptr, nullptr, m, d, face1, nullptr};
    _halfEdgeSet.insert(cm);
    _halfEdgeSet.insert(md);
    face1->halfedge = cm;
    m->halfedge = md;
    cm->next = md;
    md->next = dc;
    dc->next = cm;

    Halfedge* mc = new Halfedge{nullptr, nullptr, m, c, face2, nullptr};
    Halfedge* am = new Halfedge{nullptr, nullptr, a, m, face2, nullptr};

    _halfEdgeSet.insert(mc);
    _halfEdgeSet.insert(am);
    face2->halfedge = am;
    am->next = mc;
    mc->next = ca;
    ca->next = am;

    Halfedge* ma = new Halfedge{nullptr, nullptr, m, a, face3, nullptr};
    Halfedge* bm = new Halfedge{nullptr, nullptr, b, m, face3, nullptr};
    _halfEdgeSet.insert(ma);
    _halfEdgeSet.insert(bm);
    face3->halfedge = ma;
    ab->face = face3;
    ma->next = ab;
    ab->next = bm;
    bm->next = ma;

    Halfedge* mb = new Halfedge{nullptr, nullptr, m, b, face4, nullptr};
    Halfedge* dm = new Halfedge{nullptr, nullptr, d, m, face4, nullptr};
    _halfEdgeSet.insert(mb);
    _halfEdgeSet.insert(dm);
    face4->halfedge = mb;
    bd->face = face4;
    mb->next = bd;
    bd->next = dm;
    dm->next = mb;

    Edge* cm_edge = new Edge{cm, std::make_pair(c->index, m->index)};
    if (m->index < c->index) {
        cm_edge->vertices = std::make_pair(m->index, c->index);
    }
    cm->edge = cm_edge;
    mc->edge = cm_edge;
    Edge* md_edge = new Edge{md, std::make_pair(m->index, d->index)};
    if (d->index < m->index) {
        md_edge->vertices = std::make_pair(d->index, m->index);
    }
    md->edge = md_edge;
    dm->edge = md_edge;
    Edge* am_edge = new Edge{am, std::make_pair(a->index, m->index)};
    if (m->index < a->index) {
        am_edge->vertices = std::make_pair(m->index, a->index);
    }
    am->edge = am_edge;
    ma->edge = am_edge;
    Edge* mb_edge = new Edge{mb, std::make_pair(m->index, b->index)};
    if (b->index < m->index) {
        mb_edge->vertices = std::make_pair(b->index, m->index);
    }
    mb->edge = mb_edge;
    bm->edge = mb_edge;

    _edgeMap[cm_edge->vertices] = cm_edge;
    _edgeMap[md_edge->vertices] = md_edge;
    _edgeMap[am_edge->vertices] = am_edge;
    _edgeMap[mb_edge->vertices] = mb_edge;

    // am_edge and md_edge are the 'new' edges that subdivide needs
    newEdges->insert(am_edge);
    newEdges->insert(md_edge);

    // Glue twins together
    cm->twin = mc;
    mc->twin = cm;
    ma->twin = am;
    am->twin = ma;
    mb->twin = bm;
    bm->twin = mb;
    md->twin = dm;
    dm->twin = md;

    c->halfedge = cm;
    d->halfedge = dm;
    a->halfedge = am;
    b->halfedge = bm;

    // Delete original edge
    _halfEdgeSet.erase(cb);
    _halfEdgeSet.erase(bc);
    delete cb;
    delete bc;

    _edgeMap.erase(edge->vertices);
    delete edge;
    return m;
}

void Mesh::loopSubdivide() {
    std::unordered_map<Vertex*, Vector3f> vertexToOldPosition;
    // Make a reverse mapping from the Vertex struct to its position
    for (auto const& [key, val] : _vertexMap) {
        vertexToOldPosition[val] = val->p;
    }

    // Calculate the new positions of the old vertices
    for (auto const& [key, val] : vertexToOldPosition) {
        // key is vertex pointer, value is its old position
        Vector3f newPosition(0.f, 0.f, 0.f);
        int n = countDegree(key);
        float u = 0.f;
        if (n == 3) {
            u = 3.f/16.f;
        } else {
            float x = (3.f/8.f) + ((1.f/4.f)*std::cos((2.f*M_PI)/static_cast<float>(n)));
            u = (1.f/static_cast<float>(n))*((5.f/8.f) - std::pow(x, 2.f));
        }

        // Loop over neighbors
        Halfedge* h = key->halfedge;
        do {
            // Get old position of each neighbor
            newPosition += (vertexToOldPosition[h->destination])*u;
            h = h->twin->next;
        }
        while (h != key->halfedge);

        newPosition += (1.f - static_cast<float>(n)*u)*val;

        key->p = newPosition;
    }


    // Make copy of edges pointers to iterate over
    std::vector<Edge*> edgeCopy; // (_edgeMap.begin(), _edgeMap.end());
    for (const auto& pair : _edgeMap) {
        edgeCopy.push_back(pair.second);
    }

    // Split each edge and store a set of new Vertices created
    std::unordered_set<Edge*> newEdges;
    std::unordered_set<Vertex*> newVertices;
    for (Edge* e: edgeCopy) {
        newVertices.insert(split(e, &newEdges));
    }

    // std::cout << newEdges.size() << std::endl;

    // Flip all new edges that touch an old and new vertex
    for (Edge* e: newEdges) {
        Halfedge* h = e->halfedge;
        Vertex* s = h->source;
        Vertex* d = h->destination;
        // Either if s is old and d is new or if d is old and s is new
        if (vertexToOldPosition.contains(s) != vertexToOldPosition.contains(d)) {
            flip(e);
        }
    }

    // Calculate position for all new vertices
    for (Vertex* v: newVertices) {
        // Get the halfedge that points to an old vertex
        Halfedge* h = v->halfedge;
        do {
            h = h->twin->next;
        }
        while (!vertexToOldPosition.contains(h->destination));

        Vertex* v2 = h->destination;
        Vertex* v4 = h->next->next->twin->next->next->twin->next->destination;
        Vertex* v1 = h->next->next->twin->next->twin->next->destination;
        Vertex* v3 = h->twin->next->twin->next->next->twin->next->destination;

        Vector3f p2 = vertexToOldPosition[v2];
        Vector3f p4 = vertexToOldPosition[v4];
        Vector3f p1 = vertexToOldPosition[v1];
        Vector3f p3 = vertexToOldPosition[v3];

        v->p = (1.f/8.f)*p1 + (1.f/8.f)*p3 + (3.f/8.f)*p2 + (3.f/8.f)*p4;
    }

}

void Mesh::remesh(float w) {
    unordered_set<Edge*> edgesCopy;

    float L = 0.f;
    // Make a shallow copy of _edgeMap while calculating average length
    for (const auto& pair : _edgeMap) {
        Edge *e = pair.second;
        L += (e->halfedge->destination->p - e->halfedge->source->p).norm();
        edgesCopy.insert(e);
    }
    L = L / static_cast<float>(_edgeMap.size());

    // Split edges with length > 4/3 average length
    for (Edge* e: edgesCopy) {
        float edgeLength = (e->halfedge->destination->p - e->halfedge->source->p).norm();
        unordered_set<Edge*> newEdges; // unused
        if (edgeLength > (4.f/3.f)*L) {
            split(e, &newEdges);
        }
    }

    // Make a fresh shallow copy of the modified _edgeMap
    edgesCopy.clear();
    for (const auto& pair : _edgeMap) {
        Edge *e = pair.second;
        edgesCopy.insert(e);
    }

    for (Edge* e: edgesCopy) {
        assert(e->halfedge != nullptr);
        Halfedge* h = e->halfedge;
        assert(h->edge == e && h->twin->edge == e);
        assert(h == h->twin->twin);
    }

    // Collapse edges with length < 4/5 average length to their midpoint
    for (Edge* e: edgesCopy) {
        if (!_edgeMap.contains(e->vertices)) {
            continue;
        }
        float edgeLength = (e->halfedge->destination->p - e->halfedge->source->p).norm();
        Vector3f midPoint = (e->halfedge->source->p + e->halfedge->destination->p) / 2.f;
        Vector4f mp(midPoint[0], midPoint[1], midPoint[2], 1.f);
        if (edgeLength < (4.f/5.f)*L) {
            collapse(e, mp);
        }
    }

    // Make a fresh shallow copy of the modified _edgeMap
    edgesCopy.clear();
    for (const auto& pair : _edgeMap) {
        Edge *e = pair.second;
        edgesCopy.insert(e);
    }

    for (Edge* e: edgesCopy) {
        // Vertices currently connecting the edge
        Vertex* v1 = e->halfedge->source;
        Vertex* v2 = e->halfedge->destination;
        // Vertices to be flipped to
        Vertex* v3 = e->halfedge->next->destination;
        Vertex* v4 = e->halfedge->twin->next->destination;

        // This measures how much closer or farther we will get from both vertices having degree 6 if
        // we were to flip this edge
        int preFlipErrorV1V2 = std::abs(countDegree(v1) - 6) + std::abs(countDegree(v2) - 6);
        int postFlipErrorV1V2 = std::abs(countDegree(v1) - 6 - 1) + std::abs(countDegree(v2) - 6 - 1);
        // Are we reducing error by flipping v1, v2?
        int V1V2 = postFlipErrorV1V2 - preFlipErrorV1V2;
        // This measures how much closer or farther the two vertices being flipped to will be from having degree 6
        int preFlipErrorV3V4 = std::abs(countDegree(v3) - 6) + std::abs(countDegree(v4) - 6);
        int postFlipErrorV3V4 = std::abs(countDegree(v3) - 6 + 1) + std::abs(countDegree(v4) - 6 + 1);
        // Are we reducing error by flipping into v3, v4?
        int V3V4 = postFlipErrorV3V4 - preFlipErrorV3V4;

        if (V1V2 + V3V4 < 0) {
            // If we are reducing error overall
            flip(e);
        }
    }

    // Tangential smoothing
    for (auto const& [index, vertex] : _vertexMap) {
        Vector3f x = vertex->p;
        Vector3f n(0, 0, 0);
        Vector3f c(0, 0, 0);
        Halfedge* h = vertex->halfedge;
        int numFaces = 0;
        do {
            Vector3f v1 = h->source->p;
            Vector3f v2 = h->next->source->p;
            Vector3f v3 = h->next->next->source->p;
            n += calculateSurfaceNormal(v1, v2, v3);
            c += v2;
            numFaces++;
            h = h->twin->next;
        }
        while (h != vertex->halfedge);
        // Vertex normal calculated as average of adjacent face normals
        n = n / static_cast<float>(numFaces);
        n = n.normalized();
        // Centroid calculated as average position of all neighbors
        c = c / static_cast<float>(numFaces);

        Vector3f v = c - x;
        v = v - (n.dot(v))*n;

        vertex->p = x + v*w;
    }
}

void Mesh::validate(){
    assert(_faceMap.size() + _vertexMap.size() == _edgeMap.size()+2);
    for (Halfedge* halfedge: _halfEdgeSet) {
        // Basic null checks
        assert(halfedge->twin != nullptr);
        assert(halfedge->next != nullptr);
        assert(halfedge->source != nullptr);
        assert(halfedge->destination != nullptr);
        assert(halfedge->face != nullptr);
        assert(halfedge->face->halfedge != nullptr);
        assert(halfedge->source->halfedge != nullptr);
        assert(halfedge->destination->halfedge != nullptr);
        assert(halfedge->edge != nullptr);

        // Edge checks
        assert(halfedge->edge->halfedge == halfedge || halfedge->edge->halfedge == halfedge->twin);
        assert(halfedge->edge == halfedge->twin->edge);

        // Loop checks
        assert(halfedge->twin->twin == halfedge);
        assert(halfedge->next->next->next == halfedge);

        // Vertex checks
        assert(halfedge->source == halfedge->twin->destination);
        assert(halfedge->destination == halfedge->twin->source);
        assert(halfedge->source == halfedge->next->next->destination);

        // Face checks
        Halfedge* h = halfedge->face->halfedge;
        assert(h == halfedge || h->next == halfedge || h->next->next == halfedge);
    }

    // Vertex checks
    for (auto const& [key, val] : _vertexMap) {
        Vertex* v = val;
        assert(val->halfedge != nullptr);
        assert(val->halfedge->source == val);
        assert(val->halfedge->source == val->halfedge->twin->destination);
        // assert(val->halfedge->edge == val->halfedge->edge);
    }

    // Edge checks
    assert(_edgeMap.size() == _halfEdgeSet.size()/2);
    for (const auto& pair : _edgeMap) {
        Edge *edge = pair.second;
        assert(edge->halfedge != nullptr);
        Halfedge* h = edge->halfedge;
        assert(h->edge == edge && h->twin->edge == edge);
        assert(h == h->twin->twin);
    }

    // Face checks
    for (const auto& pair : _faceMap) {
        Face *face = pair.second;
        Halfedge* h = face->halfedge;
        assert(h != nullptr);
        do {
            assert(h->face == face);
            h = h->next;
        }
        while (h != face->halfedge);
    }
}

vector<Vector3f> Mesh::getVertices() {
    return _vertices;
}

vector<Vector3i> Mesh::getFaces() {
    return _faces;
}

const unordered_map<int, Vertex*>& Mesh::getVertexMap() {
    return _vertexMap;
}

const unordered_map<int, Face*>& Mesh::getFaceMap() {
    return _faceMap;
}

const unordered_map<pair<int, int>, Edge*, PairHash>& Mesh::getEdgeMap() {
    return _edgeMap;
}

const Vertex* Mesh::getVertex(int vertex) {
    assert(_vertexMap.contains(vertex));
    return _vertexMap[vertex];
}

const Face* Mesh::getFace(int face) {
    assert(_faceMap.contains(face));
    return _faceMap[face];
}

const Edge* Mesh::getEdge(std::pair<int, int> edge) {
    assert(_edgeMap.contains(edge));
    return _edgeMap[edge];
}
