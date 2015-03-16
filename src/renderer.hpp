#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <list>
#include <atomic>

#include "camera.hpp"
#include "scene.hpp"
#include "algebra.hpp"
#include "image.hpp"

class Camera;
class SceneNode;
class Light;
class GeometryNode; 

using namespace std;

class Renderer {
public:
    Renderer(Camera *camera, SceneNode *scene, list<Light*> lights, Colour ambient, int ssLevel)
             :  mAmbientColour(ambient), mLights(lights), mSSLevel(ssLevel), mCamera(camera), mScene(scene) {}
    
    void renderImage(const string &filename);
    Colour traceRay(const Ray &ray, int depth, const Material *sourceMaterial) const;
    bool checkVisibility(const Point3D &a, const Point3D &b) const;

    Colour mAmbientColour;
    list<Light*> mLights;

private:
    void renderSlicesThread(Image &img, int totalSlices, atomic_int &currentSlice, atomic_int &pixelCounter);
    void renderSlice(Image &img, int slice, int totalSlices, atomic_int &pixelCounter);
    Intersection findClosestIntersection(const Ray &ray) const;
    Intersection findClosestIntersectionNoLights(const Ray &ray) const;
    Colour computeColour(const Intersection &intersection) const;
    Colour backGroundColour(const Vector3D &u) const;

    int mSSLevel;

    Camera *mCamera;
    SceneNode *mScene;

    list<GeometryNode> mGeometryList;
};

#endif
