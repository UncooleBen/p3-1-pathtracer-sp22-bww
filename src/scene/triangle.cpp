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
  Vector3D n = cross(p1 - p2, p2 - p3);
  if(dot(n, r.d) ==0){
    return false;
  }
  double t = dot(p1 - r.o, n) / dot(r.d, n);
  if (t < 0 || t < r.min_t || t > r.max_t) {
    return false;
  }
  return true;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  if(!has_intersection(r)){
    return false;
  }

  float u, v;
  Vector3D n = cross(p1 - p2, p2 - p3);
  double t = dot(p1 - r.o, n) / dot(r.d, n);
  Vector3D Phit = r.o + t * r.d; 
  //find out if P is inside the triangle
  //refer to this link: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
  Vector3D edge0 = p2 - p1; 
  Vector3D vp0 = Phit - p1; 
  Vector3D Crs = cross(edge0, vp0); 
  if (dot(n,Crs) < 0) return false; // P is on the right side 

  // edge 1
  Vector3D edge1 = p3 - p2; 
  Vector3D vp1 = Phit - p2; 
  Crs = cross(edge1, vp1); 
  if ((u = dot(n,Crs)) < 0)  return false; // P is on the right side 

  // edge 2
  Vector3D edge2 = p1 - p3; 
  Vector3D vp2 = Phit - p3; 
  Crs =  cross(edge2, vp2); 
  if ((v = dot(n,Crs))< 0) return false; // P is on the right side; 


  isect->t = t;
  isect->primitive = this;
  isect->bsdf = get_bsdf();
  r.max_t = t;
  //calculate the normal of point using barycentric coordinate
  Vector3D N = cross(p2-p1, p3-p1); // N 
  float denom = dot(N,N);
  u /= denom; 
  v /= denom; 
  isect->n = u*n1 + v*n2 + (1-u-v)*n3;
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
