
#include "../rays/tri_mesh.h"
#include "../rays/samplers.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2 or 3
    // Compute the bounding box of the triangle.

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::hit.

    auto bmin =
        hmin(vertex_list[v0].position, hmin(vertex_list[v1].position, vertex_list[v2].position));
    auto bmax =
        hmax(vertex_list[v0].position, hmax(vertex_list[v1].position, vertex_list[v2].position));
    return BBox(bmin, bmax);
}

Trace Triangle::hit(const Ray& ray) const {

    // Each vertex contains a postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // TODO (PathTracer): Task 2
    // Intersect the ray with the triangle defined by the three vertices.

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)

    auto e1 = v_1.position - v_0.position;
    auto e2 = v_2.position - v_0.position;
    auto s = ray.point - v_0.position;

    auto divisor = dot(cross(e1, ray.dir), e2);
    if(std::abs(divisor) < std::numeric_limits<float>::epsilon()) return ret;

    auto uvt =
        Vec3(-dot(cross(s, e2), ray.dir), dot(cross(e1, ray.dir), s), -dot(cross(s, e2), e1)) /
        divisor;

    auto u = uvt[0];
    auto v = uvt[1];
    auto t = uvt[2];

    if(0.0f <= u && u <= 1.0f && 0.0f <= v && v <= 1.0f && u + v <= 1.0f &&
       ray.dist_bounds[0] <= t && t <= ray.dist_bounds[1]) {
        ret.hit = true;
        ret.distance = t;
        ret.position = ray.at(t);
        ret.normal = (1.0f - u - v) * v_0.normal + u * v_1.normal + v * v_2.normal;
    }

    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

Vec3 Triangle::sample(Vec3 from) const {
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    Samplers::Triangle sampler(v_0.position, v_1.position, v_2.position);
    Vec3 pos = sampler.sample();
    return (pos - from).unit();
}

float Triangle::pdf(Ray wray, const Mat4& T, const Mat4& iT) const {

    Ray tray = wray;
    tray.transform(iT);

    Trace trace = hit(tray);
    if(trace.hit) {
        trace.transform(T, iT.T());
        Vec3 v_0 = T * vertex_list[v0].position;
        Vec3 v_1 = T * vertex_list[v1].position;
        Vec3 v_2 = T * vertex_list[v2].position;
        float a = 2.0f / cross(v_1 - v_0, v_2 - v_0).norm();
        float g =
            (trace.position - wray.point).norm_squared() / std::abs(dot(trace.normal, wray.dir));
        return a * g;
    }
    return 0.0f;
}

void Tri_Mesh::build(const GL::Mesh& mesh, bool bvh) {

    use_bvh = bvh;
    verts.clear();
    triangle_bvh.clear();
    triangle_list.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    if(use_bvh) {
        triangle_bvh.build(std::move(tris), 4);
    } else {
        triangle_list = List<Triangle>(std::move(tris));
    }
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh, bool use_bvh) {
    build(mesh, use_bvh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangle_bvh = triangle_bvh.copy();
    ret.triangle_list = triangle_list.copy();
    ret.use_bvh = use_bvh;
    return ret;
}

BBox Tri_Mesh::bbox() const {
    if(use_bvh) return triangle_bvh.bbox();
    return triangle_list.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    if(use_bvh) return triangle_bvh.hit(ray);
    return triangle_list.hit(ray);
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    if(use_bvh) return triangle_bvh.visualize(lines, active, level, trans);
    return 0;
}

Vec3 Tri_Mesh::sample(Vec3 from) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.sample(from);
}

float Tri_Mesh::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.pdf(ray, T, iT);
}

} // namespace PT
