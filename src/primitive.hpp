#ifndef CS488_PRIMITIVE_HPP
#define CS488_PRIMITIVE_HPP

#include "algebra.hpp"
//#include "ray.hpp"

struct Intersection;
class Ray;

class Primitive {
public:
  virtual ~Primitive();
  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);
  virtual bool isLight() { return false; }
};

class Sphere : public Primitive {
public:
  virtual ~Sphere();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);
};

class Cube : public Primitive {
public:
  virtual ~Cube();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);
};

class Plane : public Primitive {
public:
  Plane(double radius) : mRadSq(radius*radius) {}
  Plane() : mRadSq(DBL_INF) {}

  virtual ~Plane();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);

private:
  double mRadSq;
};

class NonhierSphere : public Primitive {
public:
  NonhierSphere(const Point3D& pos, double radius)
    : m_pos(pos), m_radius(radius)
  {
  }
  virtual ~NonhierSphere();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);

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
  
  NonhierBox()
    : m_pos(Point3D(0,0,0)), m_size(1.0)
  {
  }

  virtual ~NonhierBox();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);

private:
  Point3D m_pos;
  double m_size;
};

class Torus : public Primitive {
public:
  virtual ~Torus();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);
};


class MengerSponge : public Primitive {
public:
  MengerSponge(const Point3D &pos, int level, int maxLevel, double size = 1.0);

  virtual ~MengerSponge();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);

  NonhierBox mBox;
  Point3D mPos;
  int mLevel, mMaxLevel;
  double mSize;
};

#endif
