#include "material.hpp"
#include "light.hpp"
#include "options.hpp"

Material::~Material()
{
}

Colour Material::computeColour(const Intersection &i, const Renderer *rend) const {

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

Colour PhongMaterial::computeColour(const Intersection &i, const Renderer *rend) const {
	//Start with ambient colour
	Colour finalColour = rend->mAmbientColour * m_kd;

	//Go through every light
	for (auto lightIt = rend->mLights.begin(); lightIt != rend->mLights.end(); ++lightIt) {
		Light* light = *lightIt;

		Colour lightContribution(0.0);

		//Need to do many samples for soft shadows
		for(int j = 0; j < light->num_samples; j++) {
			lightContribution += computeLightContribution(i, light, rend);	
		}

		finalColour += lightContribution / (double)(light->num_samples) * (1.0 -m_reflectivity) * (1.0 -m_transparency); //TODO should I do this?
	}

	//Reflection and refractions components
	if(m_reflectivity > MY_EPSILON) {
		finalColour += computeReflectedContribution(i, rend);
	}

	if(m_transparency > MY_EPSILON) {
		finalColour += computeRefractionContribution(i, rend); 
	}
	

	return finalColour;	
}

Colour PhongMaterial::computeLightContribution(const Intersection &i, Light *light, const Renderer *rend) const {
	Vector3D point = i.getPoint();
	Point3D lightPos = light->getSample();

	//Check shadow
	bool isVisible = rend->checkVisibility(point, lightPos);
	if(!isVisible) {
		return Colour(0.0);
	}

	Vector3D lightVec = lightPos - point;
	Vector3D L = (lightPos - point).normalized();
	Vector3D R = reflect(L, i.normal);
	Vector3D V = -i.ray->direction();

	double diffuseFact = qMax(dot(L, i.normal), 0.0f);
	double specularFact = qMax(dot(R, V), 0.0f);
	Colour surfaceColour = m_kd * diffuseFact + m_ks * pow(specularFact, m_shininess);

	//TODO combine this right
	//	return surfaceColour * light->colour * attenuation * (1.0 - material->getReflectivity())* (1.0 - material->getTransparency());
	return surfaceColour * light->getIntensity(lightVec);
}

Colour PhongMaterial::computeReflectedContribution(const Intersection &i, const Renderer *rend) const {
	//Compute reflected direction
	Vector3D reflDir = reflect(-i.ray->direction(), i.normal);
	Colour reflectColour(0.0);

	#if USE_GLOSSY_REFLECTIONS
	int samples = GLOSSY_SAMPLES;
	#else
	int samples = 1;
	#endif

	//Don't do a lot of samples if we don't have to
	if(m_glossy < MY_EPSILON) {
		samples = 1;
	}

	//Need to do many samples for glossy reflections
	for(int j = 0; j < samples; j++) {
		#if USE_GLOSSY_REFLECTIONS
		Ray reflectedRay(i.getPoint(), (reflDir + uniformRandomHemisphereUnitVec() * m_glossy).normalized()); //TODO - Should I preturb using the normal?
		#else
		Ray reflectedRay(i.getPoint(), reflDir); //TODO - Should I preturb using the normal?
		#endif

		reflectColour += rend->traceRay(reflectedRay.perturbed(0.01), i.depth + 1, this) * m_reflectivity;// * material->getDiffuse(); //TODO how to combine right
	}

	return reflectColour / (double)samples;
}

Colour PhongMaterial::computeRefractionContribution(const Intersection &i, const Renderer *rend) const {
	//http://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf

	double n1 = i.sourceMaterial->getIor(), n2 = m_ior;

	//If same source and current, then assume going into air
	if(this == i.sourceMaterial){
		n2 = 1.0;
	}

	const Vector3D &l = i.ray->direction();
	double n = n1/n2;
	double c = -dot(i.normal, l);
	double sinsq = n * n * (1.0 - c * c);

	if(n1 < n2 && sinsq > 1.0){
		return Colour(0.0);
	}

	Vector3D refr = n * l + (n * c - c) * i.normal;
	Ray refrRay(i.getPoint(), refr); //TODO - Should I preturb using the normal?material->getTransparency();

	return rend->traceRay(refrRay, i.depth + 1, this) * m_transparency * m_kd; //TODO properly * material->getDiffuse();
}

LightMaterial::~LightMaterial()
{
}

Colour LightMaterial::computeColour(const Intersection &i, const Renderer *rend) const {
	return mLight->colour;
}
