#ifndef CS488_MATERIAL_HPP
#define CS488_MATERIAL_HPP

#include "algebra.hpp"
#include "renderer.hpp"
#include "ray.hpp"
#include "light.hpp"

class Renderer;

class Material {
public:
  virtual ~Material();

  virtual Colour computeColour(const Intersection &i, const Renderer *rend) const;
  virtual double getIor() const { return 1.0; }

protected:
  Material()
  {
  }
};

class PhongMaterial : public Material {
public:
  PhongMaterial(const Colour& kd, const Colour& ks, double shininess);
  PhongMaterial(const Colour& kd, const Colour& ks, double shininess, double reflectivity, double ior = 1.0, double transparency = 0.0, double glossy = 0.0);
  virtual ~PhongMaterial();

  virtual Colour computeColour(const Intersection &i, const Renderer *rend) const;

  const Colour &getDiffuse() const { return m_kd; }
  const Colour &getSpecular() const { return m_ks; }
  
  double getReflectivity() const { return m_reflectivity; }
  double getShininess() const { return m_shininess; }
  virtual double getIor() const { return m_ior; }
  double getTransparency() const { return m_transparency; }
  double getGloss() const { return m_glossy; }

private:
  Colour computeLightContribution(const Intersection &i, Light *light, const Renderer *rend) const;
  Colour computeReflectedContribution(const Intersection &i, const Renderer *rend) const;
  Colour computeRefractionContribution(const Intersection &i, const Renderer *rend) const;

  Colour m_kd;
  Colour m_ks;

  double m_shininess, m_reflectivity, m_ior, m_transparency, m_glossy;
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
