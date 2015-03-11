#include "material.hpp"

Material::~Material()
{
}

PhongMaterial::PhongMaterial(const Colour& kd, const Colour& ks, double shininess)
  : m_kd(kd), m_ks(ks), m_shininess(shininess), m_reflectivity(0.0), m_ior(1.0), m_transparency(0.0), m_glossy(0.0)
{
}

PhongMaterial::PhongMaterial(const Colour& kd, const Colour& ks, double shininess, double reflectivity, double ior, double transparency, double glossy)
  : m_kd(kd), m_ks(ks), m_shininess(shininess), m_reflectivity(reflectivity), m_ior(ior), m_transparency(transparency), m_glossy(glossy)
{
}

PhongMaterial::~PhongMaterial()
{
}

void PhongMaterial::apply_gl() const
{
  // Perform OpenGL calls necessary to set up this material.
}
