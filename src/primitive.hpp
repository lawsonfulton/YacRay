#ifndef CS488_PRIMITIVE_HPP
#define CS488_PRIMITIVE_HPP

#include "algebra.hpp"
//#include "ray.hpp"

struct Intersection;
class Ray;

class Primitive {
public:
  virtual ~Primitive();
  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point);
};

class Sphere : public Primitive {
public:
  virtual ~Sphere();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point);
};

class Cube : public Primitive {
public:
  virtual ~Cube();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point);
};

class NonhierSphere : public Primitive {
public:
  NonhierSphere(const Point3D& pos, double radius)
    : m_pos(pos), m_radius(radius)
  {
  }
  virtual ~NonhierSphere();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point);

private:
  Point3D m_pos;
  double m_radius;
};

class NonhierBox : public Primitive {
public:
  NonhierBox(const Point3D& pos, double size)
    : m_pos(pos), m_size(size)
  {
  }
  
  virtual ~NonhierBox();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point);

private:
  Point3D m_pos;
  double m_size;
};

class Torus : public Primitive {
public:
  virtual ~Torus();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point);
};

#endif
