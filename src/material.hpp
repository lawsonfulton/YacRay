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
  void setSpecularMap(const char* filename);
  void setBumpMap(const char* filename, double magnitude);

  void setFresnel(double r0);

protected:
  Material();
  Colour getTextureColour(Point2D uv) const;
  Colour getSpecularColour(Point2D uv) const;
  double getBumpVal(Point2D uv) const;
  Vector3D getDisplacementNormal(const Intersection &i) const;

  Image *mTexmap;
  Image *mSpecmap;

  Image *mBumpmap;
  double mBumpMagnitude;

  bool mUseFresnel;
  double mR0;
};

class PhongMaterial : public Material {
public:
  PhongMaterial(const Colour& kd, const Colour& ks, double shininess);
  PhongMaterial(const Colour& kd, const Colour& ks, double shininess, double reflectivity, double ior = 1.0, double transparency = 0.0, int refSamples = 1);
  virtual ~PhongMaterial();

  virtual Colour computeColour(const Intersection &i, const Renderer *rend) const;

  const Colour &getDiffuse() const { return mKd; }
  const Colour &getSpecular() const { return mKs; }
  
  double getReflectivity() const { return mReflectivity; }
  double getShininess() const { return mShininess; }
  virtual double getIor() const { return mIor; }
  double getTransparency() const { return mTransparency; }

private:
  Colour computeLightContribution(const Vector3D &normal, const Colour &diffuseComp, const Intersection &i, Light *light, const Renderer *rend) const;
  Colour computeReflectedContribution(const Vector3D &normal, const Intersection &i, const Renderer *rend) const;
  Colour computeRefractionContribution(const Vector3D &normal, const Intersection &i, const Renderer *rend) const;
  void computeFresnelCoefs(const Intersection &i, const Vector3D &normal, double &Fr, double &Ft) const;

  Colour mKd;
  Colour mKs;

  double mShininess, mReflectivity, mIor, mTransparency;
  int mRefSamples;
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
