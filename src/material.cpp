#include "material.hpp"
#include "light.hpp"
#include "options.hpp"


Material::Material() {
	mTexmap = NULL;
	mBumpmap = NULL;
	mSpecmap = NULL;
}

Material::~Material()
{
	delete mTexmap;
	delete mBumpmap;
}


Colour Material::computeColour(const Intersection &i, const Renderer *rend) const {

}

void Material::setTextureMap(const char* filename) {
	mTexmap = new Image();
	mTexmap->loadPng(filename);
}

void Material::setSpecularMap(const char* filename) {
	mSpecmap = new Image();
	mSpecmap->loadPng(filename);
}

void Material::setBumpMap(const char* filename, double magnitude) {
	mBumpmap = new Image();
	mBumpmap->loadPng(filename);
	mBumpMagnitude = magnitude;
}

Colour Material::getTextureColour(Point2D uv) const {
	return mTexmap->bilinearGetColour(uv);
}

Colour Material::getSpecularColour(Point2D uv) const {
	return mSpecmap->bilinearGetColour(uv);
}

double Material::getBumpVal(Point2D uv) const {
	//Colour mapCol = mBumpmap->bilinearGetColour(uv);
	Colour mapCol = mBumpmap->getColour(uv);
	return (mapCol.R() + mapCol.G() + mapCol.B()) / 3.0; //TODO this might be a waste
}

Vector3D Material::getDisplacementNormal(const Intersection &i) const {
	const Point2D &uv = i.uv;

	//double E = 1.0/1000.0; //TODO how much?
	double E = 2.0/max(mBumpmap->width(),mBumpmap->height()) * 1;
	Point2D across = Point2D(E, 0.0);
	Point2D up = Point2D(0.0, E);

	Vector3D Ou, Ov;
	i.node->getTangents(i, Ou, Ov);

	double Bu = (getBumpVal(uv + across) - getBumpVal(uv - across))/(2.0 * E);
	double Bv = (getBumpVal(uv + up) - getBumpVal(uv - up))/(2.0 * E);

	Vector3D X = cross(i.normal, Ov);
	Vector3D Y = cross(i.normal, Ou);

	Vector3D D = (Bu * X - Bv * Y) * mBumpMagnitude;
	return (i.normal + D).normalized();
}

PhongMaterial::PhongMaterial(const Colour& kd, const Colour& ks, double shininess)
  : mKd(kd), mKs(ks), mShininess(shininess), mReflectivity(0.0), mIor(1.0), mTransparency(0.0), mRefSamples(1)
{
}

PhongMaterial::PhongMaterial(const Colour& kd, const Colour& ks, double shininess, double reflectivity, double ior, double transparency, int refSamples)
  : mKd(kd), mKs(ks), mShininess(shininess), mReflectivity(reflectivity), mIor(ior), mTransparency(transparency), mRefSamples(refSamples)
{
}

PhongMaterial::~PhongMaterial()
{
}

Colour PhongMaterial::computeColour(const Intersection &i, const Renderer *rend) const {
	Colour diffuseComp = mKd, specularComp = mKs;
	Vector3D normal = i.normal;
	Colour It(0.0), Ir(0.0), lightSum(0.0);
	

	//Get colour from tex map
	if(mTexmap) {
		diffuseComp = mKd * getTextureColour(i.uv);
	}

	if(mSpecmap) {
		specularComp = mKs * getSpecularColour(i.uv); //TODO switch diffuse to this technique
	}

	if(mBumpmap) {
		normal = getDisplacementNormal(i);
	}
	const Colour &Ia = rend->mAmbientColour * diffuseComp;

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
	if(specularComp.R() > MY_EPSILON || specularComp.G() > MY_EPSILON || specularComp.B() > MY_EPSILON) {
		Ir = computeReflectedContribution(normal, i, rend);
	}

	double Fr = 1.0, Ft = 1.0;
	if(mTransparency > MY_EPSILON) {
		It = computeRefractionContribution(normal, i, rend); 
		//if(i.depth == 0)computeFresnelCoefs(i, normal, Fr, Ft); //TODO fix this
		//cout << Fr << " " << Ft  << endl;
	}

	//computeFresnelCoefs(i, normal, Fr, Ft);
	double costheta = dot(-i.ray->direction(), normal);
	double R0 = 0.5;//specularComp.R();
	if(costheta > 0) {
		Fr = R0 + (1.0 - R0)*pow((1.0-costheta), 5);	
	}
	
	
	
	return Ia * diffuseComp
		   + lightSum
		   + specularComp * Fr * Ir 
		   + mTransparency * (Colour(1.0) - specularComp) * Ft * It;
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
	Colour surfaceColour = diffuseComp * diffuseFact;// + mKs * pow(specularFact, mShininess);

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

	Matrix4x4 rot;
	rot.rotate(-phi * M_180_PI, 0.0, 1.0, 0.0);
	rot.rotate(-theta * M_180_PI, 0.0, 0.0, 1.0);

	// Matrix4x4 rot;
	// rot.rotate(-90, 0.0, 0.0, 1.0);
	// rot.rotate(-90, 0.0, 1.0, 0.0);
	// cout << rot * Vector3D(0,1,0) << endl;
	// exit(1);

	//Need to do many samples for glossy reflections
	for(int j = 0; j < mRefSamples; j++) {
		double x1 = uniformRand(), x2 = uniformRand();
		double alpha = acos(pow(1.0 - x1, 1.0 / (mShininess + 1.0)));
		double beta = 2.0 * M_PI * x2;

		Vector3D perturb(sin(alpha) * cos(beta),
						 cos(alpha),
						 sin(beta) * sin(alpha));
		
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
		reflectColour += rend->traceRay(reflectedRay.perturbed(0.0001), i.depth + 1, this);// * material->getDiffuse(); //TODO how to combine right
	}

	return reflectColour / (double)mRefSamples;
}

Colour PhongMaterial::computeRefractionContribution(const Vector3D &normal, const Intersection &i, const Renderer *rend) const {
	//http://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
	Vector3D refr;
	double n1 = i.sourceMaterial->getIor(), n2 = mIor;

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
     	//return Colour(0.0);
         return rend->backGroundColour(normal);
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

	return rend->traceRay(refrRay, i.depth + 1, this) * mTransparency;// * mKd; //TODO properly * material->getDiffuse();
}

void PhongMaterial::computeFresnelCoefs(const Intersection &i, const Vector3D &normal, double &Fr, double &Ft) const {
	double n1 = i.sourceMaterial->getIor(), n2 = mIor;

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
