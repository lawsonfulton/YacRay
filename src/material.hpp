#ifndef CS488_MATERIAL_HPP
#define CS488_MATERIAL_HPP

#include "algebra.hpp"
#include "renderer.hpp"
#include "ray.hpp"
#include "light.hpp"
#include "Image.hpp"

class Renderer;

class Material {
public:
  virtual ~Material();

  virtual Colour computeColour(const Intersection &i, const Renderer *rend) const;
  virtual double getIor() const { return 1.0; }

  void setTextureMap(const char* filename);
  void setBumpMap(const char* filename);

protected:
  Material();
  Colour getTextureColour(Point2D uv) const;
  double getBumpVal(Point2D uv) const;
  Vector3D getDisplacementNormal(const Vector3D &n, const Point2D &uv) const;

  Image *m_texmap;
  Image *m_bumpmap;
};

class PhongMaterial : public Material {
public:
  PhongMaterial(const Colour& kd, const Colour& ks, double shininess);
  PhongMaterial(const Colour& kd, const Colour& ks, double shininess, double reflectivity, double ior = 1.0, double transparency = 0.0, int refSamples = 1);
  virtual ~PhongMaterial();

  virtual Colour computeColour(const Intersection &i, const Renderer *rend) const;

  const Colour &getDiffuse() const { return m_kd; }
  const Colour &getSpecular() const { return m_ks; }
  
  double getReflectivity() const { return m_reflectivity; }
  double getShininess() const { return m_shininess; }
  virtual double getIor() const { return m_ior; }
  double getTransparency() const { return m_transparency; }

private:
  Colour computeLightContribution(const Vector3D &normal, const Colour &diffuseComp,const Intersection &i, Light *light, const Renderer *rend) const;
  Colour computeReflectedContribution(const Vector3D &normal, const Intersection &i, const Renderer *rend) const;
  Colour computeRefractionContribution(const Vector3D &normal, const Intersection &i, const Renderer *rend) const;
  void computeFresnelCoefs(const Intersection &i, const Vector3D &normal, double &Fr, double &Ft) const;

  Colour m_kd;
  Colour m_ks;

  double m_shininess, m_reflectivity, m_ior, m_transparency;
  int m_refSamples;
};

class LightMaterial {
public:
  LightMaterial(const Light *light) : mLight(light) {}
  virtual ~LightMaterial();

  virtual Colour computeColour(const Intersection &i, const Renderer *rend) const;
  virtual double getIor() const { return 1.0; }

protected:
  const Light *mLight;
};

//class GlossyMaterial : public Material

#endif
