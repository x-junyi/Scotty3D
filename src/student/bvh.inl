
#include "../rays/bvh.h"
#include "debug.h"
#include <array>
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    struct Bucket {
        BBox bbox{};
        size_t prim_count{};
    };

    constexpr size_t n_buckets = 8;
    std::stack<size_t> node_stack;
    BBox root_box;
    for(const auto& prim : primitives) root_box.enclose(prim.bbox());
    root_idx = new_node(root_box, 0, primitives.size(), 0, 0);
    node_stack.push(root_idx);

    while(!node_stack.empty()) {
        auto node_idx = node_stack.top();
        auto& node = nodes[node_idx];
        node_stack.pop();
        if(node.size <= max_leaf_size) continue;

        int best_partition_axis{};
        float best_partition_coord{};
        float best_partition_cost = std::numeric_limits<float>::max();
        BBox best_partition_l_bbox{};
        BBox best_partition_r_bbox{};
        auto node_end = node.start + node.size;

        for(int axis = 0; axis < 3; ++axis) {
            std::array<Bucket, n_buckets> buckets{};
            auto bbox_length = node.bbox.max[axis] - node.bbox.min[axis];
            for(auto i = node.start; i < node_end; ++i) {
                auto bidx =
                    size_t(floor((primitives[i].bbox().center()[axis] - node.bbox.min[axis]) /
                                 bbox_length * n_buckets));
                if(bidx == n_buckets) bidx = bidx - 1;
                buckets[bidx].bbox.enclose(primitives[i].bbox());
                buckets[bidx].prim_count++;
            }

            BBox l_bbox{}, r_bbox{};
            size_t l_cnt{}, r_cnt{};
            for(int i = 0; i + 1 < n_buckets; ++i) {
                l_bbox.enclose(buckets[i].bbox);
                l_cnt += buckets[i].prim_count;
                r_bbox.reset();
                r_cnt = 0;
                for(int j = i + 1; j < n_buckets; ++j) {
                    r_bbox.enclose(buckets[j].bbox);
                    r_cnt += buckets[j].prim_count;
                }
                auto cost =
                    (l_bbox.surface_area() * float(l_cnt) + r_bbox.surface_area() * float(r_cnt));
                if(cost < best_partition_cost) {
                    best_partition_axis = axis;
                    best_partition_cost = cost;
                    best_partition_coord =
                        node.bbox.min[axis] + (float(i) + 1.0f) * bbox_length / n_buckets;
                }
            }
        }

        size_t bound =
            std::partition(primitives.begin() + node.start, primitives.begin() + node_end,
                           [best_partition_axis = best_partition_axis,
                            best_partition_coord = best_partition_coord](const Primitive& p) {
                               return p.bbox().center()[best_partition_axis] < best_partition_coord;
                           }) -
            primitives.begin();
        if(node.start == bound || bound == node_end) continue;

        for(auto i = node.start; i < bound; ++i)
            best_partition_l_bbox.enclose(primitives[i].bbox());
        for(auto i = bound; i < node_end; ++i) best_partition_r_bbox.enclose(primitives[i].bbox());
        nodes[node_idx].l = new_node(best_partition_l_bbox, node.start, bound - node.start, 0, 0);
        nodes[node_idx].r = new_node(best_partition_r_bbox, bound, node_end - bound, 0, 0);
        node_stack.push(nodes[node_idx].l);
        node_stack.push(nodes[node_idx].r);
    }
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    std::stack<size_t> node_stack;
    node_stack.push(root_idx);

    while(!node_stack.empty()) {
        auto node_idx = node_stack.top();
        node_stack.pop();
        auto& node = nodes[node_idx];

        Vec2 times0{};
        auto hit0 = node.bbox.hit(ray, times0);
        if(!hit0 || (ret.hit && ret.distance <= times0.x)) continue;

        if(node.is_leaf()) {
            auto node_end = node.start + node.size;
            for(auto i = node.start; i < node_end; ++i) {
                Trace hit = primitives[i].hit(ray);
                ret = Trace::min(ret, hit);
            }
        } else {
            Vec2 times1, times2;
            auto hit1 = nodes[node.l].bbox.hit(ray, times1);
            auto hit2 = nodes[node.r].bbox.hit(ray, times2);

            if(hit1 && hit2) {
                auto first = times1.x < times2.x ? node.l : node.r;
                auto second = times1.x < times2.x ? node.r : node.l;
                node_stack.push(first);
                node_stack.push(second);
            } else if(hit1) {
                node_stack.push(node.l);
            } else if(hit2) {
                node_stack.push(node.r);
            }
        }
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
