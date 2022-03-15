#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();
  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;
  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  Vector3D L_p_wr;
  for(int i= 0; i < num_samples; i++){
    Vector3D w_in = hemisphereSampler->get_sample();
    if(w_out.z <0){
      w_in.z = -w_in.z;
    }
    //calculation at object coordinate 
    Vector3D bsdf_f = isect.bsdf->f(w_out, w_in);
    Ray w_in_ray = Ray(hit_p, o2w*w_in);
    w_in_ray.min_t = EPS_F;
    Intersection in_isect;

    if( bvh->intersect(w_in_ray, &in_isect)){
      Vector3D L_in = in_isect.bsdf->get_emission();
      
      Vector3D cos_thetaj = abs(w_in.z);
      L_p_wr += bsdf_f*L_in*cos_thetaj;
    }

  }
   L_p_wr =  L_p_wr/(num_samples/(2*PI));

  return L_p_wr;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
  int num_samples;
  for (auto light: scene->lights){
    int unit_light_samples = 0;
    if(light->is_delta_light()){
      unit_light_samples +=1;
    }
    else{
      unit_light_samples += ns_area_light;
    }
    num_samples += unit_light_samples;

    for(int i= 0; i < unit_light_samples; i++){
      Vector3D world_w_in;  
      double disttolight, pdf;
      Vector3D L_in =  light->sample_L(hit_p, &world_w_in, &disttolight, &pdf);
      //calculation at object coordinate
      Vector3D w_in = w2o*world_w_in; 
      Vector3D bsdf_f = isect.bsdf->f(w_out, w_in);

      Ray w_in_ray = Ray(hit_p, world_w_in);
      w_in_ray.min_t = EPS_F;
      w_in_ray.max_t =  disttolight/world_w_in.norm() - EPS_F;

     //Intersection in_isect;
      if(!bvh->has_intersection(w_in_ray)){
      Vector3D cos_thetaj = abs(w_in.z);
      L_out += bsdf_f*L_in*cos_thetaj/pdf;
      }
    }
  }
  L_out = L_out/num_samples;
  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if( direct_hemisphere_sample){
    return estimate_direct_lighting_hemisphere(r, isect);
  }
  else{
    return estimate_direct_lighting_importance(r, isect);
  }

}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);
  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  Vector3D w_in;
  double pdf;
  double prob = 0.7;
      //calculation at object coordinate
  

  Vector3D bsdf_f = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  if ((r.depth == max_ray_depth) || (r.depth > 0 && coin_flip(prob))) {
      L_out += one_bounce_radiance(r, isect);
      Vector3D world_w_in = o2w*w_in;
      Ray w_in_ray = Ray(hit_p , world_w_in, (int)r.depth-1);
      w_in_ray.min_t = EPS_F;
      Intersection in_isect;
      if(bvh->intersect(w_in_ray, &in_isect)){
        Vector3D L_in = in_isect.bsdf->get_emission();
        Vector3D cos_thetaj = abs(w_in.z);
        L_out += at_least_one_bounce_radiance(w_in_ray,in_isect)*pdf;//*L_in*cos_thetaj/pdf/prob;
      }
  }
  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;
  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  L_out += zero_bounce_radiance(r, isect); 
  //L_out += one_bounce_radiance(r, isect);
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  L_out += at_least_one_bounce_radiance(r, isect);
  // parts of global illumination into L_out rather than just direct


  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
 // To estimate the integral of radiance over a pixel, 
 //you should generate ns_aa random rays using generate_ray(...) which you implemented in part 1.
 // For each ray, you should call PathTracer::est_radiance_global_illumination(Ray r) to estimate the scene radiance along that ray and then incorporate it into the Monte Carlo estimate of the Vector3D value of the pixel.

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D radiance_per_pixel;
  for (int i = 0; i < num_samples; i++){
    Vector2D pixelpos = gridSampler->get_sample() + origin;
    Ray unitray = camera->generate_ray(pixelpos[0]/sampleBuffer.w, pixelpos[1]/sampleBuffer.h);
    unitray.depth = max_ray_depth;
    radiance_per_pixel += est_radiance_global_illumination(unitray);
  }
  radiance_per_pixel = radiance_per_pixel/num_samples;
  sampleBuffer.update_pixel(radiance_per_pixel, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
