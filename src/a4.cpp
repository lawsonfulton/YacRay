#include "a4.hpp"
#include "renderer.hpp"

#include <chrono>

using namespace std;
using namespace std::chrono;

void a4_render(// What to render
               SceneNode* root,
               // Where to output the image
               const std::string& filename,
               // Image size
               int width, int height,
               // Viewing parameters
               const Point3D& eye, const Vector3D& view,
               const Vector3D& up, double fov,
               // Lighting parameters
               const Colour& ambient,
               const std::list<Light*>& lights
               )
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  cout << "Rendering: " << filename << "  " << width << " x " << height << endl;
  cout << "Camera Location: " << toString(eye) << " Look at: " << toString(view) << " Up: " << toString(up) << " FOV: " << fov << endl; 

  Camera camera(eye, view, up, fov, width, height);
  Renderer renderer(camera, root, lights, ambient); 

  renderer.renderImage(filename);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

  cout << "Running time: " << duration/1000000.0 << endl;
}
