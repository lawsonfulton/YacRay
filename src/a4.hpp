#ifndef CS488_A4_HPP
#define CS488_A4_HPP

#include <string>
#include "scene.hpp"
#include "light.hpp"

void a4_render(// What to render
               SceneNode* root,
               // Where to output the image
               const std::string& filename,
               // Image size
               int width, int height,
               int ss_level,
               // Viewing parameters
               const Point3D& eye, const Vector3D& view,
               const Vector3D& up, double fov,
               // Lighting parameters
               const Colour& ambient,
               const std::list<Light*>& lights,
               const char *skymap
               );

#endif
