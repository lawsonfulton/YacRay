#include "material.hpp"
#include "light.hpp"
#include "options.hpp"


Material::Material() {
	m_texmap = NULL;
	m_bumpmap = NULL;
}

Material::~Material()
{
	delete m_texmap;
	delete m_bumpmap;
}


Colour Material::computeColour(const Intersection &i, const Renderer *rend) const {

}

void Material::setTextureMap(const char* filename) {
	m_texmap = new Image();
	m_texmap->loadPng(filename);
}

void Material::setBumpMap(const char* filename) {
	m_bumpmap = new Image();
	m_bumpmap->loadPng(filename);
}

Colour Material::getTextureColour(Point2D uv) const {
	return m_texmap->bilinearGetColour(uv);
}

double Material::getBumpVal(Point2D uv) const {
	Colour mapCol = m_bumpmap->bilinearGetColour(uv);
	return (mapCol.R() + mapCol.G() + mapCol.B()) / 3.0; //TODO this might be a waste
}

Vector3D Material::getDisplacementNormal(const Vector3D &n, const Point2D &uv) const {
	double E = 1.0/64.0; //TODO how much?
 	//Question - should i be offsetting uv values or actual physical point and then getting new uv values?
	Point2D across = Point2D(E, 0.0);
	Point2D up = Point2D(0.0, E);

	double Bu = (getBumpVal(uv + across) - getBumpVal(uv - across))/(2.0 * E);
	double Bv = (getBumpVal(uv + up) - getBumpVal(uv - up))/(2.0 * E);

	return Bu * Vector3D(1.0,0.0,0.0) + Bv * Vector3D(0.0,1.0,0.0); //TODO properly calc X and Y
}

PhongMaterial::PhongMaterial(const Colour& kd, const Colour& ks, double shininess)
  : m_kd(kd), m_ks(ks), m_shininess(shininess), m_reflectivity(0.0), m_ior(1.0), m_transparency(0.0), m_refSamples(1)
{
}

PhongMaterial::PhongMaterial(const Colour& kd, const Colour& ks, double shininess, double reflectivity, double ior, double transparency, int refSamples)
  : m_kd(kd), m_ks(ks), m_shininess(shininess), m_reflectivity(reflectivity), m_ior(ior), m_transparency(transparency), m_refSamples(refSamples)
{
}

PhongMaterial::~PhongMaterial()
{
}

Colour PhongMaterial::computeColour(const Intersection &i, const Renderer *rend) const {
	Colour diffuseComp = m_kd;
	Vector3D normal = i.normal;
	Colour It(0.0), Ir(0.0), lightSum(0.0);
	const Colour &Ia = rend->mAmbientColour;

	//Get colour from tex map
	if(m_texmap) {
		diffuseComp = getTextureColour(i.uv);
	}

	if(m_bumpmap) {
		normal = (i.normal + getDisplacementNormal(i.normal, i.uv)).normalized();
	}

	//Go through every light
	for (auto lightIt = rend->mLights.begin(); lightIt != rend->mLights.end(); ++lightIt) {
		Light* light = *lightIt;

		Colour lightContribution(0.0);

		//Need to do many samples for soft shadows
		for(int j = 0; j < light->num_samples; j++) {
			lightContribution += computeLightContribution(normal, diffuseComp, i, light, rend);	
		}

		lightSum += lightContribution / (double)(light->num_samples); //TODO should I do this?
	}

	//Reflection and refractions components
	if(m_ks.R() > MY_EPSILON && m_ks.G() > MY_EPSILON && m_ks.B() > MY_EPSILON) {
		Ir = computeReflectedContribution(normal, i, rend);
	}

	double Fr = 1.0, Ft = 1.0;
	if(m_transparency > MY_EPSILON) {
		It = computeRefractionContribution(normal, i, rend); 
		//if(i.depth == 0)computeFresnelCoefs(i, normal, Fr, Ft); //TODO fix this
		//cout << Fr << " " << Ft  << endl;
	}

	
	
	
	return Ia * diffuseComp
		   + lightSum
		   + m_ks * Fr * Ir 
		   + m_transparency * (Colour(1.0) - m_ks) * Ft * It;
/**	I = Ka * Ia
+ Kd * [sum for each light: (N . L) * Il]
+ Ks * [sum for each light: ((R . V) ^ Ps) * Fl * Il]
+ Ks * Fr * Ir
+ Kt * (1 - Ks) * Ft * It

I := surface point's color
V := ray direction
P := surface point
N := surface normal
L := light's position - P
R := L - 2 * (N . L) * P
Ka := surface material's ambient coefficient
Kd := surface material's diffuse coefficient
Ks := surface material's specular coefficient
Ps := surface material's shininess
Kt := surface material's transmission coefficient
Ia := ambient light color
Il := light's color
Ir := reflected ray's color
It := transmitted ray's color
Fl := light's Fresnel coefficient
Fr := reflected Fresnel coefficient
Ft := transmitted Fresnel coefficient**/
}

//TODO refactor so no passing in intersection
Colour PhongMaterial::computeLightContribution(const Vector3D &normal, const Colour &diffuseComp, const Intersection &i, Light *light, const Renderer *rend) const {
	Vector3D point = i.getPoint();
	Point3D lightPos = light->getSample();

	//Check shadow
	bool isVisible = rend->checkVisibility(point, lightPos);
	if(!isVisible) {
		return Colour(0.0);
	}

	Vector3D lightVec = lightPos - point;
	Vector3D L = (lightPos - point).normalized();
	Vector3D R = reflect(L, normal);
	Vector3D V = -i.ray->direction();

	double diffuseFact = qMax(dot(L, normal), 0.0f);
	double specularFact = qMax(dot(R, V), 0.0f);
	Colour surfaceColour = diffuseComp * diffuseFact;// + m_ks * pow(specularFact, m_shininess);

	//TODO combine this right
	//	return surfaceColour * light->colour * attenuation * (1.0 - material->getReflectivity())* (1.0 - material->getTransparency());
	return surfaceColour * light->getIntensity(lightVec);
}

Colour PhongMaterial::computeReflectedContribution(const Vector3D &normal, const Intersection &i, const Renderer *rend) const {
	//Compute reflected direction
	Vector3D reflDir = reflect(-i.ray->direction(), normal);
	Colour reflectColour(0.0);

	double theta = acos(reflDir.y());
	double phi = atan2(reflDir.z(), reflDir.x());

	// Matrix4x4 rot;
	// rot.rotate(-90, 0.0, 0.0, 1.0);
	// rot.rotate(-90, 0.0, 1.0, 0.0);
	// cout << rot * Vector3D(0,1,0) << endl;
	// exit(1);

	//Need to do many samples for glossy reflections
	for(int j = 0; j < m_refSamples; j++) {
		double x1 = uniformRand(), x2 = uniformRand();
		double alpha = acos(pow(1.0 - x1, 1.0 / (m_shininess + 1.0)));
		double beta = 2.0 * M_PI * x2;

		Vector3D perturb(sin(alpha) * cos(beta),
						 cos(alpha),
						 sin(beta) * sin(alpha));

		Matrix4x4 rot;
		rot.rotate(-phi * M_180_PI, 0.0, 1.0, 0.0);
		rot.rotate(-theta * M_180_PI, 0.0, 0.0, 1.0);
		
		// cout << "offset: " << perturb << " refDir: " << reflDir;
		reflDir = rot * perturb;
		// cout << " newRefDir: " << reflDir << endl;

		// cout << reflDir << " ";
		// reflDir = Vector3D(sin(theta) * cos(phi),
		// 				   cos(theta),
		// 				   sin(phi) * sin(theta));

		//reflDir =  origOffset * Vector3D(0.0, 1.0, 0.0);
		// cout << reflDir << endl;

		Ray reflectedRay(i.getPoint(), reflDir);
		reflectColour += rend->traceRay(reflectedRay.perturbed(0.01), i.depth + 1, this);// * material->getDiffuse(); //TODO how to combine right
	}

	return reflectColour / (double)m_refSamples;
}

Colour PhongMaterial::computeRefractionContribution(const Vector3D &normal, const Intersection &i, const Renderer *rend) const {
	//http://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
	Vector3D refr;
	double n1 = i.sourceMaterial->getIor(), n2 = m_ior;

	Vector3D n = normal;
	//If same source and current, then assume going into air
	if(this == i.sourceMaterial){
		n2 = 1.0;
		n = -n;
	}

	 const Vector3D &v = i.ray->direction();
     double k = dot(v, n);
 
     Vector3D s = (n1/n2) * (v - k*n);
 
     k = 1.0 - dot(s,s);
 
     if (k < MY_EPSILON)
     {
     	//cout << k << endl;
         return Colour(0.0);
     }
     else
     {
     	//cout << "ay" << endl;
        refr = s - sqrt(k)*n;
     }

	// const Vector3D &l = i.ray->direction();
	// double n = n1/n2;
	// double c = -dot(normal, l);
	// double sinsq = n * n * (1.0 - c * c);

	// if(n1 < n2 && sinsq > 1.0){
	// 	return Colour(0.0);
	// }

	// Vector3D refr = n * l + (n * c - c) * normal;
	Ray refrRay(i.getPoint(), refr.normalized()); //TODO - Should I preturb using the normal?material->getTransparency();

	return rend->traceRay(refrRay, i.depth + 1, this) * m_transparency;// * m_kd; //TODO properly * material->getDiffuse();
}

void PhongMaterial::computeFresnelCoefs(const Intersection &i, const Vector3D &normal, double &Fr, double &Ft) const {
	double n1 = i.sourceMaterial->getIor(), n2 = m_ior;

	Vector3D norm = normal;
	const Vector3D &v = i.ray->direction();
	//If same source and current, then assume going into air
	if(this == i.sourceMaterial){
		n2 = 1.0;
		norm = -norm;
	}

	double n = n1 / n2;
	double cosThetaI = dot(v, norm) * -1.0;
	double sinThetaI = sqrt(1.0 - cosThetaI*cosThetaI); //TODO Recomputing these, should just do all calculations in one function
	double sinThetaT = n * sinThetaI;
	double cosThetaT = sqrt(1.0 - sinThetaT*sinThetaT);

	double a = n1 * cosThetaI;
	double b = n2 * cosThetaT;
	double Rs = (a - b) / (a + b);
	Rs = Rs * Rs;

	a = n1 * cosThetaT;
	b = n2 * cosThetaI;
	double Rt = (a - b) / (a + b);
	Rt = Rt * Rt;

	Fr = (Rs + Rt) / 2.0;
	Ft = 1.0 - Fr;
}

LightMaterial::~LightMaterial()
{
}

Colour LightMaterial::computeColour(const Intersection &i, const Renderer *rend) const {
	return mLight->colour;
}
