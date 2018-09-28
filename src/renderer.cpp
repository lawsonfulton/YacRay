#include <iomanip>
#include <list>
#include <thread>

#include "renderer.hpp"
#include "material.hpp"
#include "scene.hpp"
#include "options.hpp"

// *** Things To Test ***
// - Get rid of pertubs and do proper t transformation check
// - combine colours correctly
// - Better background

static inline void loadbar(unsigned int x, unsigned int n, unsigned int w = 50)
{
	//Source: https://www.ross.click/2011/02/creating-a-progress-bar-in-c-or-any-other-console-app/
    if ( (x != n) && (x % (n/100+1) != 0) ) return;
 
    float ratio  =  x/(float)n;
    int   c      =  ratio * w;
 
    cout << setw(3) << (int)(ratio*100) << "% [";
    for (int x=0; x<c; x++) cout << "=";
    for (int x=c; x<(int)w; x++) cout << " ";
    cout << "]\r" << flush;
}

Renderer::Renderer(Camera *camera, SceneNode *scene, 
		            list<Light*> lights, Colour ambient,
		            int ssLevel,int dofSamples, double aperature, double focalLen, 
		            const char* skymap, bool useTone, double Lwhite,  double a)
             :  mAmbientColour(ambient), mLights(lights), mSSLevel(ssLevel),
             mDofSamples(dofSamples), mAperature(aperature), mFocalLen(focalLen),
             mCamera(camera), mScene(scene), mSkymap(NULL),
             mUseToneMap(useTone), mToneLwhite(Lwhite), mToneA(a) {
    if(skymap) {
    	mSkymap = new Image();
    	cout << "Loading environment map..." << flush;
    	mSkymap->loadPng(skymap);
    	cout << " Done." << endl;
    }

    //Cache for adaptive anti aliasing
	mRayColours.resize(camera->width() + 1);
	for(int i = 0; i < (int)mRayColours.size(); i++) {
		mRayColours[i].resize(camera->height() + 1, NULL);   
	}

	mSourceMaterial = (Material*) new PhongMaterial(Colour(0.0), Colour(0.0), 0, 1.0); //Air
}

Renderer::~Renderer() {
	for(int i = 0; i < (int)mRayColours.size(); i++) {
		for(int j = 0; j < (int)mRayColours[i].size(); j++) {
			delete mRayColours[i][j];
		}
	}

	delete mSourceMaterial;
	delete mSkymap;
}

void Renderer::renderImage(const string &filename) {
	cout << endl;//formattting
	Image img(mCamera->width(), mCamera->height(), 3);

	//Flatten the DAG
	mGeometryList = mScene->getFlattened();

	//Add the lights into the list of objects so we can see them
	for (auto lightIt = mLights.begin(); lightIt != mLights.end(); ++lightIt) {
		LightMaterial *lightMaterial = new LightMaterial(*lightIt);
		GeometryNode lightNode("light", *lightIt);
		lightNode.set_material((Material*)lightMaterial);
		mGeometryList.push_back(lightNode);
	}

	vector<thread> threads;
	atomic_int pixelCounter(0);
	atomic_int currentSlice(0); 
	int totalSlices = 8 * NUM_THREADS;
	cout << "Number of Threads: " << NUM_THREADS << endl;
	cout << "Number of Image Slices: " << totalSlices << endl;

	for (int i = 0; i < NUM_THREADS; i++) {
		threads.push_back(thread(&Renderer::renderSlicesThread,
								 this, std::ref(img), totalSlices, std::ref(currentSlice), std::ref(pixelCounter)));
	}

	for (auto& th : threads) th.join();

	if(mUseToneMap) {
		cout << endl;
		cout << "Tone mapping image..." << flush;
		img.ReinhardToneMap(mToneLwhite, mToneA);
		cout << "Done." << endl;
	}

	img.savePng(filename);
	cout << endl;//formatting
}

void Renderer::renderSlicesThread(Image &img, int totalSlices, atomic_int &currentSlice, atomic_int &pixelCounter) {
	int slice;

	while((slice = currentSlice++) < totalSlices) {
		renderSlice(img, slice, totalSlices, pixelCounter);
	}
}

// void Renderer::renderSlice(Image &img, int slice, int totalSlices, atomic_int &pixelCounter) {
// 	int totalPixels = mCamera->width() * mCamera->height();

// 	int sliceWidth = mCamera->width() / totalSlices;
// 	int start = slice * sliceWidth;
// 	int end = start + sliceWidth;

// 	//Get any extra pixels
// 	if(slice == totalSlices - 1) {
// 		end += mCamera->width() % totalSlices;
// 	}

// 	for (int y = 0; y < mCamera->height(); y++) {
// 		for (int x = start; x < end; x++) {

// 			Colour finalColour = computePixelColour(x,y);
// 			img.setColour(x, y, finalColour);
			
// 			//Update the loading bar
// 			pixelCounter++;
// 			loadbar(pixelCounter, totalPixels);
// 		}
// 	}
// }

Colour Renderer::computePixelColour(int x, int y) {
	double numSamples = 0;
	double spacing = 1.0/(double)mSSLevel;

	Colour finalColour(0.0);

	//Test the corners
	if(!mRayColours[x][y]) {
		Ray primaryRay = mCamera->makeRay(Point2D((double)x, (double)y));
		mRayColours[x][y] = new Colour(traceRay(primaryRay, 0, mSourceMaterial));
	}
	if(!mRayColours[x + 1][y]) {
		Ray primaryRay = mCamera->makeRay(Point2D((double)x + 1, (double)y));
		mRayColours[x + 1][y] = new Colour(traceRay(primaryRay, 0, mSourceMaterial));
	}
	if(!mRayColours[x][y + 1]) {
		Ray primaryRay = mCamera->makeRay(Point2D((double)x, (double)y + 1));
		mRayColours[x][y + 1] = new Colour(traceRay(primaryRay, 0, mSourceMaterial));
	}
	if(!mRayColours[x + 1][y + 1]) {
		Ray primaryRay = mCamera->makeRay(Point2D((double)x + 1, (double)y + 1));
		mRayColours[x + 1][y + 1] = new Colour(traceRay(primaryRay, 0, mSourceMaterial));
	}

	Colour &c0 = *mRayColours[x][y];
	Colour &c1 = *mRayColours[x + 1][y];
	Colour &c2 = *mRayColours[x][y + 1];
	Colour &c3 = *mRayColours[x + 1][y + 1];

	finalColour += c0;
	finalColour += c1;
	finalColour += c2;
	finalColour += c3;
	numSamples += 4;

	double eps = 0.02;
	if(c0.almostEqual(c1, eps) && c0.almostEqual(c2, eps) && c0.almostEqual(c3, eps)) {
		finalColour = finalColour / 4.0;
	}
	else {
		int samples;
		finalColour += subSample(Point2D(x, y + 1), Point2D(x + 1, y), samples);
		numSamples += samples;
		finalColour = finalColour / numSamples;
		//finalColour = Colour(1.0,0.0,0.0);
	}
	

	// for(int yi = 0; yi < mSSLevel; yi++) {
	// 	for(int xi = 0; xi < mSSLevel; xi++) {
	// 		Ray primaryRay = mCamera->makeRay(Point2D((double)x + xi * spacing, (double)y + yi * spacing));
	// 		Colour colour = traceRay(primaryRay, 0, (Material*)mSourceMaterial);

	// 		finalColour += colour / numSamples;
	// 	}
	// }

	return finalColour;
}

Colour Renderer::subSample(const Point2D &minP, const Point2D &maxP, int &nSamples) {
	Colour finalColour(0.0);
	Ray primaryRay;
	double d = (maxP.x() - minP.x()) / 2.0;

	//middle top
	primaryRay = mCamera->makeRay(Point2D(minP.x() + d, maxP.y()));
	finalColour += traceRay(primaryRay, 0, mSourceMaterial);
	//left 
	primaryRay = mCamera->makeRay(Point2D(minP.x(), maxP.y() + d));
	finalColour += traceRay(primaryRay, 0, mSourceMaterial);
	//middle 
	primaryRay = mCamera->makeRay(Point2D(minP.x() + d, maxP.y() + d));
	finalColour += traceRay(primaryRay, 0, mSourceMaterial);
	//right 
	primaryRay = mCamera->makeRay(Point2D(maxP.x(), maxP.y() + d));
	finalColour += traceRay(primaryRay, 0, mSourceMaterial);
	//middle bottom 
	primaryRay = mCamera->makeRay(Point2D(minP.x() + d, minP.y()));
	finalColour += traceRay(primaryRay, 0, mSourceMaterial);

	nSamples = 5;
	return finalColour;
}

void Renderer::renderSlice(Image &img, int slice, int totalSlices, atomic_int &pixelCounter) {
	double numSamples = (double)(mSSLevel * mSSLevel);
	double spacing = 1.0/(double)mSSLevel;

	int totalPixels = mCamera->width() * mCamera->height();

	int sliceWidth = mCamera->width() / totalSlices;
	int start = slice * sliceWidth;
	int end = start + sliceWidth;

	//Get any extra pixels
	if(slice == totalSlices - 1) {
		end += mCamera->width() % totalSlices;
	}

	PhongMaterial sourceMaterial(Colour(0.0), Colour(0.0), 0, 1.0); //Air
	for (int y = 0; y < mCamera->height(); y++) {
		for (int x = start; x < end; x++) {

			Colour finalColour(0.0);

			for(int yi = 0; yi < mSSLevel; yi++) {
				for(int xi = 0; xi < mSSLevel; xi++) {
					Point2D initialPoint = Point2D((double)x + xi * spacing, (double)y + yi * spacing);
					Ray primaryRay = mCamera->makeRay(initialPoint);

					if(mAperature > 0.0) {
						Colour dofColour(0.0);

						Point3D focalPoint = primaryRay.getPoint(mFocalLen);
						const Vector3D &viewDir = mCamera->getDirection();
						double viewTheta = acos(viewDir.y());
						double viewPhi = atan2(viewDir.z(), viewDir.x());

						Matrix4x4 rot;
						rot.rotate(-viewPhi * M_180_PI, 0.0, 1.0, 0.0);
						rot.rotate(-viewTheta * M_180_PI, 0.0, 0.0, 1.0);

						for(int i = 0; i < mDofSamples; i++) {
							double theta = uniformAngleVal();
							double r = uniformRand() * mAperature;
							
							Vector3D offset(sqrt(r) * cos(theta), 0, sqrt(r) * sin(theta));
							offset = rot * offset;

							Point3D newOrigin = mCamera->getLocation() + offset;
							Vector3D newDir = (focalPoint - newOrigin).normalized();

							Ray dofRay(newOrigin, newDir);

							Colour colour = traceRay(dofRay, 0, (Material*)&sourceMaterial);
							dofColour += colour;
						}

						finalColour += dofColour / mDofSamples;
					}
					else {
						Ray primaryRay = mCamera->makeRay(Point2D((double)x + xi * spacing, (double)y + yi * spacing));
						Colour colour = traceRay(primaryRay, 0, (Material*)&sourceMaterial);

						finalColour += colour;
					}
				}
			}

			finalColour = finalColour / numSamples;

			img.setColour(x, y, finalColour);
			
			//Update the loading bar
			pixelCounter++;
			loadbar(pixelCounter, totalPixels);
		}
	}
}

Colour Renderer::traceRay(const Ray &ray, int depth, const Material *sourceMaterial) const {
	if(depth > MAX_RECURSION_DEPTH) {
		return backGroundColour(ray.direction()); //TODO should I return ambient?
	}
	Intersection closest = findClosestIntersection(ray);
	closest.sourceMaterial = sourceMaterial;
	closest.depth = depth;

	return computeColour(closest);
}


Intersection Renderer::findClosestIntersection(const Ray &ray, bool includeLights) const {
	Intersection closestI;
	closestI.ray = &ray;

	for (auto nodeIt = mGeometryList.begin(); nodeIt != mGeometryList.end(); ++nodeIt) {
		if(includeLights || !(*nodeIt).isLight()) {
			Intersection newI;
			bool intersects = nodeIt->computeIntersection(ray, newI); //TODO should I perturb here? //ray.perturbed(MY_EPSILON)

			if(intersects) {
				if(newI.t < closestI.t && newI.t > MIN_INTERSECT_DIST) {
					closestI = newI;
				}
			}
		}
	}

	return closestI;
}

Colour Renderer::computeColour(const Intersection &i) const {
	if(i.t == DBL_INF) {
		return backGroundColour(i.ray->direction());
	}

	const Material *material = i.node->get_material();
	return material->computeColour(i, this);
}

Colour Renderer::backGroundColour(const Vector3D &direction) const {
	if(!mSkymap) {
		return Colour(0.0);
	}

	double u = 0.5 + (-M_PI/2.0 + atan2(direction.z(), -direction.x())) / (2.0 * M_PI);
	double v = 0.5 - (asin(direction.y()))/ M_PI;
	Point2D uv(u,v);

	return mSkymap->bilinearGetColour(uv);
	
	//Sky sphere
	// double theta = asin(direction.y());
	// double t = (theta + M_PI_2)/M_PI;

	// Colour horizon(1,1,1);///(0.8,0.8,0.9);
	// Colour bottom(1,1,1);
	// Colour top(0,0,1);

	// if(t > 0.5) {
	// 	return lin_interpolate(horizon, top, clamp((t - 0.5) * 8.0, 0.0, 1.0));
	// } else {
	// 	return lin_interpolate(bottom, horizon, t * 2.0);
	// }
}



//TODO should I also preturb here? Should I preturb in find closest intersection?
// or is it already taken care of with the MIN_DIST component
bool Renderer::checkVisibility(const Point3D &a, const Point3D &b) const {
	double dist = (b-a).length();

	Ray shadowRay(a, (b-a)/dist);
	Intersection i = findClosestIntersection(shadowRay.perturbed(0.0001), false); //TODO this perturbment is a bit of a hack, should really be doing proer calculation of t in world space

	if(i.t == DBL_INF) {
		return true;
	} else {
		return i.t > dist; //Return true if intersection is past the light
	}
}

