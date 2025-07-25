#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
    BBox bbox;
    for (auto it = start; it != end; ++it) {
        bbox.expand((*it)->get_bbox());
    }

    BVHNode* node = new BVHNode(bbox);

    size_t num_primitives = end - start;
    if (num_primitives <= max_leaf_size) {
        // Leaf node
        node->start = start;
        node->end = end;
        return node;
    }

    BBox centroid_bbox;
    for (auto it = start; it != end; ++it) {
        centroid_bbox.expand((*it)->get_bbox().centroid());
    }

  
    Vector3D extent = centroid_bbox.extent;
    int axis = 0;
    if (extent.y > extent.x && extent.y > extent.z)
        axis = 1;
    else if (extent.z > extent.x && extent.z > extent.y)
        axis = 2;


    double split_point = 0.0;
    for (auto it = start; it != end; ++it) {
        split_point += ((*it)->get_bbox().centroid())[axis];
    }
    split_point /= num_primitives;

    auto mid = std::partition(start, end, [axis, split_point](Primitive* p) {
        return p->get_bbox().centroid()[axis] < split_point;
        });

    if (mid == start || mid == end) {
        mid = start + (num_primitives / 2);
    }

    // Recursively build children
    node->l = construct_bvh(start, mid, max_leaf_size);
    node->r = construct_bvh(mid, end, max_leaf_size);

    return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

    if (!node) node = root;                      

    double t0, t1;
    if (!node->bb.intersect(ray, t0, t1)) return false;

    if (node->isLeaf()) {
        for (auto it = node->start; it != node->end; ++it) {
            ++total_isects;
            if ((*it)->has_intersection(ray)) return true;  
        }
        return false;
    }

    return has_intersection(ray, node->l) ||
        has_intersection(ray, node->r);


}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.



    if (!node) node = root;

    double t0, t1;
    if (!node->bb.intersect(ray, t0, t1)) return false;

    if (node->isLeaf()) {
        bool hit = false;
        for (auto it = node->start; it != node->end; ++it) {
            ++total_isects;
            if ((*it)->intersect(ray, i)) hit = true;  
        }
        return hit;
    }

    double l_t0, l_dummy, r_t0, r_dummy;
    bool l_hit_box = node->l->bb.intersect(ray, l_t0, l_dummy);
    bool r_hit_box = node->r->bb.intersect(ray, r_t0, r_dummy);

    if (!l_hit_box && !r_hit_box) return false;

    BVHNode* first = (l_hit_box && (!r_hit_box || l_t0 <= r_t0)) ? node->l : node->r;
    BVHNode* second = (first == node->l) ? node->r : node->l;

    bool hit = intersect(ray, i, first);

    if (second) {
        double t0s, t1s;
        if (second->bb.intersect(ray, t0s, t1s)) {
            if (intersect(ray, i, second)) hit = true;
        }
    }

    return hit;
}

} // namespace SceneObjects
} // namespace CGL
