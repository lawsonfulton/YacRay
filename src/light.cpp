#include "light.hpp"
#include "ray.hpp"
#include <iostream>


Light::Light()
  : colour(0.0, 0.0, 0.0),
    position(0.0, 0.0, 0.0)
{
  falloff[0] = 1.0;
  falloff[1] = 0.0;
  falloff[2] = 0.0;
  num_samples = 1;
}

double Light::computeAttenuation(const Point3D &lightVec) {
  //Attenuation / Falloff TODO performance check
  double attenuation = 1.0;
  double denom = 0.0;
  double r2 = lightVec.lengthSquared();
  if(falloff[0] > MY_EPSILON) {
    denom += falloff[0];
  }
  if(falloff[1] > MY_EPSILON) {
    denom += falloff[1] * sqrt(r2);
  }
  if(falloff[2] > MY_EPSILON) {
    denom += falloff[2] * r2;
  }
  if(denom > MY_EPSILON) {
    attenuation = 1.0/denom;
  }

  return attenuation;
}

Point3D Light::getSample() const {
  return position;
}

Colour Light::getIntensity(const Vector3D &lightVec) {
  return colour * computeAttenuation(lightVec);
}

bool Light::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
  return false;
}

RectLight::RectLight(double x_len, double z_len, const Point3D &pos, double *atn, Colour col, int n_samples)
{
  num_samples = n_samples;
  colour = col;
  position = pos;
  falloff[0] = atn[0];
  falloff[1] = atn[1];
  falloff[2] = atn[2];

  bottomLeft = position - Vector3D(x_len/2.0, 0.0, -z_len/2.0);
  basis[0] = Vector3D(0.0,0.0,-z_len);
  basis[1] = Vector3D(x_len,0.0,0.0);
}

Colour RectLight::getIntensity(const Point3D &lightVec) {
  Vector3D toPoint = (-lightVec).normalized(); //TODO should we return the cos?

  double cosTheta = dot(toPoint,Vector3D(0,-1,0));

  if(cosTheta < 0.0) {
    return Colour(0.0);
  }

  return colour * cosTheta * computeAttenuation(lightVec);
}


Point3D RectLight::getSample() const {
  double u = uniformRand();
  double v = uniformRand();

  return bottomLeft + u * basis[0] + v * basis[1];
}

bool RectLight::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
  return false;//TODO temp
  Vector3D toPlane = position - ray.origin();
  normal = Vector3D(0,-1,0);

  t = dot(toPlane, normal) / dot(ray.direction(), normal);
  point = ray.getPoint(t);

  if(point.x() > bottomLeft.x() && point.x() < bottomLeft.x() + basis[1].x()
     && point.z() < bottomLeft.z() && point.z() > bottomLeft.z() + basis[0].z()) {
      if(t > MIN_INTERSECT_DIST) {
        return true;
      }
  }

  return false;
}

std::ostream& operator<<(std::ostream& out, const Light& l)
{
  out << "L[" << l.colour << ", " << toString(l.position) << ", ";
  for (int i = 0; i < 3; i++) {
    if (i > 0) out << ", ";
    out << l.falloff[i];
  }
  out << "]";
  return out;
}

