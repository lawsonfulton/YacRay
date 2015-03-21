#ifndef CS488_LIGHT_HPP
#define CS488_LIGHT_HPP

#include "algebra.hpp"
#include "primitive.hpp"
#include <iosfwd>

// Represents a simple point light.
class Light : public Primitive {
public:
  Light();

  virtual bool isLight() { return true; }
  
  virtual Point3D getSample() const;
  virtual Colour getIntensity(const Point3D &lightVec);
  double computeAttenuation(const Vector3D &lightVec);
  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);

  Colour colour;
  Point3D position;
  double falloff[3];
  int num_samples;
};

//bool GeometryNode::computeIntersection(const Ray &ray, Intersection &i) 
//Check visible so we can tell side

class RectLight : public Light {
	public:
		RectLight(double x_len, double z_len, const Point3D &pos, double *atn, Colour col, int n_samples);

		virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);
		
		virtual Point3D getSample() const;
		virtual Colour getIntensity(const Point3D &lightVec);
		Vector3D basis[2]; // z |__ x
		Point3D bottomLeft;
};


std::ostream& operator<<(std::ostream& out, const Light& l);

#endif
