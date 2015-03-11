#ifndef CS488_MATERIAL_HPP
#define CS488_MATERIAL_HPP

#include "algebra.hpp"

class Material {
public:
  virtual ~Material();
  virtual void apply_gl() const = 0;

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

  virtual void apply_gl() const;

  const Colour &getDiffuse() const { return m_kd; }
  const Colour &getSpecular() const { return m_ks; }
  
  double getReflectivity() const { return m_reflectivity; }
  double getShininess() const { return m_shininess; }
  double getIor() const { return m_ior; }
  double getTransparency() const { return m_transparency; }
  double getGloss() const { return m_glossy; }

private:
  Colour m_kd;
  Colour m_ks;

  double m_shininess, m_reflectivity, m_ior, m_transparency, m_glossy;
};


#endif
