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

	img.savePng(filename);
	cout << endl;//formatting
}

void Renderer::renderSlicesThread(Image &img, int totalSlices, atomic_int &currentSlice, atomic_int &pixelCounter) {
	int slice;

	while((slice = currentSlice++) < totalSlices) {
		renderSlice(img, slice, totalSlices, pixelCounter);
	}
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
					Ray primaryRay = mCamera->makeRay(Point2D((double)x + xi * spacing, (double)y + yi * spacing));
					Colour colour = traceRay(primaryRay, 0, (Material*)&sourceMaterial);

					finalColour += colour / numSamples;
				}
			}
			img.setColour(x, y, finalColour);
			
			//Update the loading bar
			pixelCounter++;
			loadbar(pixelCounter, totalPixels);
		}
	}
}

Colour Renderer::traceRay(const Ray &ray, int depth, const Material *sourceMaterial) const {
	if(depth > MAX_RECURSION_DEPTH) {
		return Colour(0.0); //TODO should I return ambient?
	}
	Intersection closest = findClosestIntersection(ray);
	closest.sourceMaterial = sourceMaterial;
	closest.depth = depth;

	return computeColour(closest);
}

Intersection Renderer::findClosestIntersection(const Ray &ray) const {
	Intersection closestI;
	closestI.ray = &ray;

	for (auto nodeIt = mGeometryList.begin(); nodeIt != mGeometryList.end(); ++nodeIt) {
		Intersection newI;
		bool intersects = nodeIt->computeIntersection(ray, newI); //TODO should I perturb here? //ray.perturbed(MY_EPSILON)

		if(intersects) {
			if(newI.t < closestI.t && newI.t > MIN_INTERSECT_DIST) {
				closestI = newI;
			}
		}
	}

	return closestI;
}

//HACK HACK HACK
Intersection Renderer::findClosestIntersectionNoLights(const Ray &ray) const {
	Intersection closestI;
	closestI.ray = &ray;

	for (auto nodeIt = mGeometryList.begin(); nodeIt != mGeometryList.end(); ++nodeIt) {
		if(!(*nodeIt).isLight()) {
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

Colour Renderer::backGroundColour(const Vector3D &u) const {
	return Colour(0.0);
	
	//Sky sphere
	double theta = asin(u.y());
	double t = (theta + M_PI_2)/M_PI;

	Colour horizon(1,1,1);///(0.8,0.8,0.9);
	Colour bottom(1,1,1);
	Colour top(0,0,1);

	if(t > 0.5) {
		return lin_interpolate(horizon, top, clamp((t - 0.5) * 8.0, 0.0, 1.0));
	} else {
		return lin_interpolate(bottom, horizon, t * 2.0);
	}
}



//TODO should I also preturb here? Should I preturb in find closest intersection?
// or is it already taken care of with the MIN_DIST component
bool Renderer::checkVisibility(const Point3D &a, const Point3D &b) const {
	double dist = (b-a).length();

	Ray shadowRay(a, (b-a)/dist);
	Intersection i = findClosestIntersectionNoLights(shadowRay.perturbed(0.01)); //TODO this perturbment is a bit of a hack, should really be doing proer calculation of t in world space

	if(i.t == DBL_INF) {
		return true;
	} else {
		return i.t > dist; //Return true if intersection is past the light
	}
}

