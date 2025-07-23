#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
	Vector3D u = p2 - p1, v = p3 - p1;
	Vector3D n = cross(u, v);
	if (dot(n, r.d) == 0)
		return false;
	double t = dot(p1 - r.o, n) / dot(r.d, n);
	if (t<r.min_t || t>r.max_t)
		return false;
	Vector3D w = r.o + t * r.d - p1;
	double d00 = dot(u, u),d01 = dot(u, v),d11 = dot(v, v),d20 = dot(w, u),d21 = dot(w, v);
	double denom = d00 * d11 - d01 * d01;

	double a = (d11 * d20 - d01 * d21) / denom;
	double b = (d00 * d21 - d01 * d20) / denom;
	double c = 1.0 - a - b;
	if (!((a >= 0) && (b >= 0) && (c >= 0)))
		return false;
	r.max_t = t;
	return true;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
	Vector3D u = p2 - p1, v = p3 - p1;
	Vector3D n = cross(u, v);
	if (dot(n, r.d) == 0)
		return false;
	double t = dot(p1 - r.o, n) / dot(r.d, n);
	if (t<r.min_t || t>r.max_t)
		return false;
	Vector3D w = r.o + t * r.d - p1;
	double d00 = dot(u, u), d01 = dot(u, v), d11 = dot(v, v), d20 = dot(w, u), d21 = dot(w, v);
	double denom = d00 * d11 - d01 * d01;

	double a = (d11 * d20 - d01 * d21) / denom;
	double b = (d00 * d21 - d01 * d20) / denom;
	double c = 1.0 - a - b;
	if (!((a >= 0) && (b >= 0) && (c >= 0)))
		return false;
	r.max_t = t;
	isect->t = t;
	isect->n = a * n1 + b * n2 + c * n3;
	isect->primitive = this;
	isect->bsdf = get_bsdf();
	return true;


}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
