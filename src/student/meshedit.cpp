
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/* 
    This method splits the given edge in half, but does not split the
    adjacent faces. Returns an iterator to the new vertex which splits
    the original edge.

    Example function for how to go about implementing local operations
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {

    // Phase 1: collect all elements
    HalfedgeRef h = (e->halfedge()->is_boundary()) ? e->halfedge()->twin() : e->halfedge();
    HalfedgeRef ht = h->twin();
    HalfedgeRef preh = h;
    HalfedgeRef nexht = ht->next();
    do {
        preh = preh->next();
    } while (preh->next() != h);
    Vec3 vpos = (h->vertex()->pos + ht->vertex()->pos)/2;

    // Phase 2: Allocate new elements
    VertexRef c = new_vertex();
    c->pos = vpos;
    HalfedgeRef hn = new_halfedge();
    HalfedgeRef hnt = new_halfedge();
    EdgeRef e0 = new_edge();

    // The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = new_face();
    HalfedgeRef h_not_used = new_halfedge();

    // Phase 3: Reassign elements
    e0->halfedge() = hn;
    hn->twin() = hnt;
    hn->edge() = e0;
    hn->vertex() = h->vertex();
    hn->face() = h->face();
    preh->next() = hn;
    hn->next() = h;
    h->vertex() = c;
    ht->next() = hnt;
    c->halfedge() = h;
    hn->vertex()->halfedge() = hn;
    // is_new parameter is used for global operations
    c->is_new = true;

    // example of set_neighbors:
    // condenses hnt->next() = nexht; hnt->twin() = hn; hnt->vertex() = c; hnt->edge() = e0; hnt->face() = ht->face(); into one line
    hnt->set_neighbors(nexht, hn, c, e0, ht->face());

    // Phase 4: Delete unused elements
    erase(f_not_used);
    erase(h_not_used);

    // Phase 5: Return the correct iterator
    return c;
}

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    // collect elements
    auto h0 = e->halfedge();
    auto h1 = h0->twin();
    auto h2 = h0->next();
    auto h3 = h1->next();
    auto h4 = h2;
    while(h4->next() != h0) h4 = h4->next();
    auto h5 = h3;
    while(h5->next() != h1) h5 = h5->next();
    auto h6 = h2->twin();
    auto h7 = h3->twin();
    auto h8 = h4->twin();
    auto h9 = h5->twin();

    auto v0 = h0->vertex();
    auto v1 = h2->vertex();
    auto v2 = h4->vertex();
    auto v3 = h5->vertex();

    auto e1 = h2->edge();
    auto e2 = h3->edge();
    auto e3 = h4->edge();
    auto e4 = h5->edge();

    auto f0 = h0->face();
    auto f1 = h1->face();

    // reassign elements
    h2->vertex() = v0;
    h4->next() = h2;
    h5->next() = h3;
    for(auto h = h2->twin()->next(); h != h3; h = h->twin()->next()) {
        h->vertex() = v0;
    }

    v0->halfedge() = h8;
    v0->pos = (v0->pos + v1->pos) / 2;

    f0->halfedge() = h2;
    f1->halfedge() = h3;

    // delete unused elements
    erase(h0);
    erase(h1);
    erase(v1);
    erase(e);

    // handle face with degree less than 3 after collapse
    if(f0->degree() < 3) {
        h6->twin() = h8;
        h6->edge() = e3;
        h8->twin() = h6;
        v2->halfedge() = h6;
        e3->halfedge() = h6;
        erase(h2);
        erase(h4);
        erase(e1);
        erase(f0);
    }

    if(f1->degree() < 3) {
        h7->twin() = h9;
        h7->edge() = e4;
        h9->twin() = h7;
        v3->halfedge() = h7;
        e4->halfedge() = h7;
        erase(h3);
        erase(h5);
        erase(e2);
        erase(f1);
    }

    return v0;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge counter-clockwise and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    if(e->on_boundary()) return std::nullopt;

    auto h0 = e->halfedge();
    auto h1 = h0->twin();
    auto h2 = h0->next();
    auto h3 = h1->next();
    auto h4 = h2->next();
    auto h5 = h3->next();
    auto h6 = h4;
    while(h6->next() != h0) h6 = h6->next();
    auto h7 = h5;
    while(h7->next() != h1) h7 = h7->next();

    auto v0 = h0->vertex();
    auto v1 = h2->vertex();
    auto v2 = h4->vertex();
    auto v3 = h5->vertex();

    auto f0 = h0->face();
    auto f1 = h1->face();

    h0->set_neighbors(h4, h1, v3, e, f0);
    h1->set_neighbors(h5, h0, v2, e, f1);
    h2->next() = h1;
    h2->face() = f1;
    h3->next() = h0;
    h3->face() = f0;
    h6->next() = h3;
    h7->next() = h2;

    v0->halfedge() = h3;
    v1->halfedge() = h2;

    f0->halfedge() = h0;
    f1->halfedge() = h1;

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    // collect elements
    auto h0 = e->halfedge();
    auto h1 = h0->twin();
    auto h2 = h0->next();
    auto h3 = h1->next();
    auto h4 = h2->next();
    auto h5 = h3->next();

    auto v0 = h0->vertex();
    auto v1 = h1->vertex();
    auto v2 = h4->vertex();
    auto v3 = h5->vertex();

    auto e1 = h2->edge();
    auto e2 = h3->edge();
    auto e3 = h4->edge();
    auto e4 = h5->edge();

    auto f0 = h0->face();
    auto f1 = h1->face();

    // triangle meshes only
    if(f0->degree() != 3 || f1->degree() != 3) {
        return std::nullopt;
    }

    // allocate new elements
    auto h6 = new_halfedge();
    auto h7 = new_halfedge();
    auto h8 = new_halfedge();
    auto h9 = new_halfedge();
    auto h10 = new_halfedge();
    auto h11 = new_halfedge();

    auto v4 = new_vertex();

    auto e5 = new_edge();
    auto e6 = new_edge();
    auto e7 = new_edge();

    auto f2 = new_face();
    auto f3 = new_face();

    // reassign elements
    h0->set_neighbors(h8, h10, v0, e, f0);
    h1->set_neighbors(h9, h11, v1, e5, f1);
    h2->set_neighbors(h6, h2->twin(), v1, e1, f2);
    h3->set_neighbors(h7, h3->twin(), v0, e2, f3);
    h4->set_neighbors(h0, h4->twin(), v2, e3, f0);
    h5->set_neighbors(h1, h5->twin(), v3, e4, f1);
    h6->set_neighbors(h11, h8, v2, e6, f2);
    h7->set_neighbors(h10, h9, v3, e7, f3);
    h8->set_neighbors(h4, h6, v4, e6, f0);
    h9->set_neighbors(h5, h7, v4, e7, f1);
    h10->set_neighbors(h3, h0, v4, e, f3);
    h11->set_neighbors(h2, h1, v4, e5, f2);

    v4->halfedge() = h10;
    v4->pos = (v0->center() + v1->center()) / 2;

    e->halfedge() = h0;
    e5->halfedge() = h1;
    e6->halfedge() = h6;
    e7->halfedge() = h7;

    f0->halfedge() = h0;
    f1->halfedge() = h1;
    f2->halfedge() = h2;
    f3->halfedge() = h3;

    return v4;
}


/*
    This method should insets a vertex into the given face, returning a pointer to the new center vertex
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
    (void)f;
    return std::nullopt;
}

/*
    This method should inset a face into the given face, returning a pointer to the new face.
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::inset_face(Halfedge_Mesh::FaceRef f) {

    // hint: use bevel_face positions as a helper function here
    (void)f;
    return std::nullopt;
}

/*
    This method should bevel a vertex and inserts a vertex into the new vertex, returning a pointer to that vertex
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::extrude_vertex(VertexRef v) {
    (void)v;
    return std::nullopt;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determines the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these 
    positions, as well as the normal and tangent offset fields to assign positions to 
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f0) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    int d = 1;
    std::vector<HalfedgeRef> old_h{f0->halfedge()};
    while(old_h.back()->next() != old_h.front()) {
        old_h.emplace_back(old_h.back()->next());
        ++d;
    }

    std::vector<HalfedgeRef> h(4 * d);
    std::vector<VertexRef> v(d);
    std::vector<EdgeRef> e(2 * d);
    std::vector<FaceRef> f(d);

    for(int i = 0; i < d; ++i) {
        h[4 * i] = new_halfedge();
        h[4 * i + 1] = new_halfedge();
        h[4 * i + 2] = new_halfedge();
        h[4 * i + 3] = new_halfedge();
        v[i] = new_vertex();
        e[2 * i] = new_edge();
        e[2 * i + 1] = new_edge();
        f[i] = new_face();
    }

    for(int b = 0; b < d; ++b) {
        auto a = (b + d - 1) % d;
        auto c = (b + 1) % d;

        auto h0 = old_h[b];
        auto v0 = h0->vertex();
        auto v1 = old_h[c]->vertex();
        auto e0 = h0->edge();

        h0->set_neighbors(h[4 * b], h0->twin(), v0, e0, f[b]);
        h[4 * b]->set_neighbors(h[4 * b + 1], h[4 * c + 2], v1, e[2 * b], f[b]);
        h[4 * b + 1]->set_neighbors(h[4 * b + 2], h[4 * b + 3], v[c], e[2 * b + 1], f[b]);
        h[4 * b + 2]->set_neighbors(h0, h[4 * a], v[b], e[2 * a], f[b]);
        h[4 * b + 3]->set_neighbors(h[4 * c + 3], h[4 * b + 1], v[b], e[2 * b + 1], f0);
        v[b]->halfedge() = h[4 * b + 3];
        v[b]->pos = v0->pos;
        e[2 * b]->halfedge() = h[4 * b];
        e[2 * b + 1]->halfedge() = h[4 * b + 1];
        f[b]->halfedge() = h0;
    }

    f0->halfedge() = h[3];

    return f0;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 0, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    auto d = new_halfedges.size();
    Vec3 normal_vec{};
    for(int i = 0; i < d; ++i) {
        Vec3 pi = start_positions[i];
        Vec3 pj = start_positions[(i + 1) % d];
        normal_vec += cross(pi, pj);
    }
    normal_vec.normalize();

    for(uint32_t b = 0; b < d; ++b) {
        auto a = (b + d - 1) % d;
        auto c = (b + 1) % d;
        auto tangent_vec = start_positions[a] + start_positions[c] - 2 * start_positions[b];
        new_halfedges[b]->vertex()->pos =
            start_positions[b] - (tangent_offset * tangent_vec + normal_offset * normal_vec);
    }
}

/*
    Updates the position of v using the given start_position
*/
void Halfedge_Mesh::extrude_vertex_position(const Vec3& start_positions, Halfedge_Mesh::FaceRef face) {
    (void)start_positions;
    (void)face;
}

/******************************************************************
*********************** Global Operations *************************
******************************************************************/

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    auto n_triangulation = n_faces();
    auto face = faces_begin();
    std::vector<HalfedgeRef> face_halfedges;
    while(n_triangulation--) {
        if(face->degree() <= 3) continue;

        auto h = face->halfedge();
        do {
            face_halfedges.push_back(h);
            h = h->next();
        } while(h != face->halfedge());

        auto n = face_halfedges.size() - 1;
        auto m = n * 0;

        while(m + 2 < n) {
            auto a = n;
            if((n + m) % 2) {
                a = n;
                m++;
            } else {
                a = n - 1;
                n--;
            }

            auto h0 = face_halfedges[a];
            if(h0->face() != face) h0 = h0->next()->next()->twin();
            auto h1 = h0->next();
            auto h2 = h1->next();
            auto h3 = face_halfedges[a - 1];
            auto v0 = h0->vertex();
            auto v1 = h1->vertex();
            auto v2 = h2->vertex();
            auto f0 = face;

            auto h4 = new_halfedge();
            auto h5 = new_halfedge();
            auto e0 = new_edge();
            auto f1 = new_face();

            h0->face() = f1;
            h1->next() = h4;
            h1->face() = f1;
            h3->next() = h5;
            h4->set_neighbors(h0, h5, v2, e0, f1);
            h5->set_neighbors(h2, h4, v0, e0, f0);

            e0->halfedge() = h4;
            f0->halfedge() = h5;
            f1->halfedge() = h4;
        }
        face_halfedges.clear();
        ++face;
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for(auto& v : vertices) {
        v.new_pos = v.pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(auto& e : edges) {
        e.new_pos = e.center();
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(auto& f : faces) {
        f.new_pos = f.center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for(auto& f : faces) {
        f.new_pos = f.center();
    }

    // Edges
    for(auto& e : edges) {
        e.new_pos = (e.halfedge()->vertex()->pos + e.halfedge()->twin()->vertex()->pos +
                     e.halfedge()->face()->new_pos + e.halfedge()->twin()->face()->new_pos) /
                    4;
    }

    // Vertices
    for(auto& v : vertices) {
        auto n = v.pos[0] * 0;
        auto Q = v.pos * 0;
        auto R = Q;
        auto S = v.pos;

        auto h = v.halfedge();
        do {
            n += 1.0;
            Q += h->face()->new_pos;
            R += h->edge()->center();
            h = h->twin()->next();
        } while(h != v.halfedge());

        Q /= n;
        R /= n;

        v.new_pos = (Q + R * 2 + S * (n - 3)) / n;
    }
}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.
    
    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    
    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.
    
    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting 
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new. 
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)
    
    // Now flip any new edge that connects an old and new vertex.
    
    // Finally, copy new vertex positions into the Vertex::pos.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
