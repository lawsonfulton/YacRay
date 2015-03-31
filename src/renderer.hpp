#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <list>
#include <vector>
#include <atomic>

#include "camera.hpp"
#include "scene.hpp"
#include "algebra.hpp"
#include "image.hpp"
#include "material.hpp"

class Camera;
class SceneNode;
class Light;
class GeometryNode; 

using namespace std;

class Renderer {
public:
    Renderer(Camera *camera, SceneNode *scene, list<Light*> lights, Colour ambient, int ssLevel,int dofSamples, double aperature, double focalLen, const char* skymap);
    ~Renderer();

    void renderImage(const string &filename);
    Colour traceRay(const Ray &ray, int depth, const Material *sourceMaterial) const;
    bool checkVisibility(const Point3D &a, const Point3D &b) const;
    Colour backGroundColour(const Vector3D &direction) const;

    Colour mAmbientColour;
    list<Light*> mLights;

private:
    void renderSlicesThread(Image &img, int totalSlices, atomic_int &currentSlice, atomic_int &pixelCounter);
    void renderSlice(Image &img, int slice, int totalSlices, atomic_int &pixelCounter);
    Colour computePixelColour(int x, int y);
    Colour subSample(const Point2D &minP, const Point2D &maxP, int &nSamples);
    Intersection findClosestIntersection(const Ray &ray, bool includeLights = true) const;
    Colour computeColour(const Intersection &intersection) const;

    int mSSLevel;
    int mDofSamples;
    double mAperature;
    double mFocalLen;
    

    Camera *mCamera;
    SceneNode *mScene;

    list<GeometryNode> mGeometryList;

    vector<vector<Colour*> > mRayColours;
    Material *mSourceMaterial;
    Image *mSkymap;
};

#endif
