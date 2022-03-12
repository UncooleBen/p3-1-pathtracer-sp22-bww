#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;
int axisid;
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
bool sortcomp(const Vector3D& v1, const Vector3D& v2)
{
  if(v1[axisid] != v2[axisid]){
     return v1[axisid] < v2[axisid];
  }
  else if(v1[(axisid+1)%3] != v2[(axisid+1)%3]){
    return v1[(axisid+1)%3] < v2[(axisid+1)%3];
  }
  else{
    return v1[(axisid+2)%3] < v2[(axisid+2)%3];
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
  //First, compute the bounding box of a list of primitives and initialize a new BVHNode with that bounding box. 
  //If there are no more than max_leaf_size primitives in the list, the node we just created is a leaf node 
  //and we should update its start and end iterators appropriately. (Please think carefully about how you would update the iterators.) Return this leaf node to end the recursion.

  BBox bbox;
  Vector3D midpointup = (*start)->get_bbox().centroid();
  Vector3D midpointdw = (*start)->get_bbox().centroid();
  vector<Vector3D> midpoints;
  int count = end - start;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    Vector3D boxcenter = (*p)->get_bbox().centroid();
    for(int i = 0; i < 3; i++){
      midpointup[i] = max(midpointup[i], boxcenter[i]);
      midpointdw[i] = min(midpointdw[i], boxcenter[i]);
    }
    midpoints.push_back(boxcenter);
    bbox.expand(bb);
  }
  // get the range of each idx
  Vector3D delta = midpointup - midpointdw;

  if(delta[1] >= delta[0] && delta[1] >= delta[2]){
    axisid = 1;
  }
  else if(delta[2] >= delta[0] && delta[2] >= delta[1]){
    axisid = 2;
  }
  //sort to find the mid point
  sort(midpoints.begin(), midpoints.end(), sortcomp);
  Vector3D midpoint = midpoints.at(count/2);
  // for(auto pm: midpoints){
  //   cout << pm << " ";
  // }
 // cout << endl;
  
  BVHNode *node = new BVHNode(bbox);
  // pure leaf node
  if(start - end <= max_leaf_size){
    node->start = start;
    node->end = end;
  }
  else{
    vector<Primitive*> leftpmt, rightpmt;
    int cntleft, cntright ;
    for (auto p = start; p != end; p++) {
      Vector3D boxcenter = (*p)->get_bbox().centroid();
      //cout << boxcenter[axisid] << " " << midpoint[axisid];
      if (boxcenter[axisid] < midpoint[axisid]){
        //eftbox.expand((*p)->get_bbox());
        if(cntleft < count/2){
          leftpmt.push_back(*p);
          cntleft +=1;
        }else{
          rightpmt.push_back(*p);
          cntright +=1;
        }
      }else{
        //rightbox.expand((*p)->get_bbox());
        if(cntright < count/2){
          rightpmt.push_back(*p);
          cntright +=1;
        }else{
          leftpmt.push_back(*p);
          cntleft +=1;
        }
      }
    }
    cout << count <<" "<< leftpmt.size() << " "<< rightpmt.size() << endl;
    node->l = construct_bvh(leftpmt.begin(), leftpmt.end(), max_leaf_size);
    node->r = construct_bvh(rightpmt.begin(), rightpmt.end(), max_leaf_size);
  }
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.



  for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
  }
  return false;


}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.



  bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit;


}

} // namespace SceneObjects
} // namespace CGL
