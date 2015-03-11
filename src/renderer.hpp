#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <list>
#include <atomic>

#include "camera.hpp"
#include "scene.hpp"
#include "light.hpp"
#include "algebra.hpp"
#include "image.hpp"

using namespace std;

class Renderer {
public:
    Renderer(Camera camera, SceneNode *scene, list<Light*> lights, Colour ambient)
             : mCamera(camera), mScene(scene), mLights(lights), mAmbientColour(ambient) {}
    
    void renderImage(const string &filename);

private:
    void renderSlicesThread(Image &img, int totalSlices, atomic_int &currentSlice, atomic_int &pixelCounter);
    void renderSlice(Image &img, int slice, int totalSlices, atomic_int &pixelCounter);

    Colour traceRay(const Ray &ray, int depth, Material *sourceMaterial);
    Intersection findClosestIntersection(const Ray &ray);
    Colour computeColour(const Intersection &intersection, int depth);
    Colour backGroundColour(const Vector3D &u);
    Colour computeLightContribution(const Intersection &i, Light *light);
    Colour computeReflectedContribution(const Intersection &i, int depth);
    Colour computeRefractionContribution(const Intersection &i, int depth);
    bool checkVisibility(const Point3D &a, const Point3D &b);

    Camera mCamera;
    SceneNode *mScene;
    list<Light*> mLights;
    Colour mAmbientColour;

    list<GeometryNode> mGeometryList;
};

#endif
