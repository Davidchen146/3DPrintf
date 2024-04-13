## Mesh (milestone submission)

Please fill this out and submit your work to Gradescope by the milestone deadline.

### Mesh Validator
Describe what your mesh validator checks for here. This can be a list of assertions.

My mesh is based on a half-edge data structure, where each half-edge contains pointers 
to the vertex, edge, and face it is associated with and is a circular linked list. Three 
half-edges are connected in a loop that defines a face.
Each half-edge also has a twin half-edge
that connects faces together. Finally, I keep track of a list of vertices and faces. 
Vertices are defined by a coordinate and one of the half-edges pointing out of it. 
Faces are defined only by one of the half-edges that make up a face.
Finally, Edges are also defined only by one of the half-edges pointing to it.

Assertions for each half-edge:
1. its next pointer is not null
2. its twin is not null
3. its vertex pointer is not null (both source and destination)
4. its face pointer is not null
5. the face associated with the half-edge has a non-null half-edge pointer
6. the vertices associated with the half-edge has a non-null half-edge pointer
7. The half-edge's twin's twin is itself
8. The half-edge's next->next->next pointer is itself (half-edges are linked in a loop)
9. The half-edge's source vertex is the same as its twin's destination vertex
10. The half-edge's source vertex is the same as its next->next->destination vertex

Assertions for each face:
1. Its half-edge pointer is not null
2. Its half-edge pointer's face pointer is itself

Assertions for each edge:
1. Its half-edge pointer is not null
2. Its half-edge's Edge pointer is itself
3. Its half-edge's twin's Edge pointer is itself

### Collaboration/References
None

### Known Bugs
None 
