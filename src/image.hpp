#ifndef CS488_IMAGE_HPP
#define CS488_IMAGE_HPP

#include <string>
#include "algebra.hpp"

/** An image, consisting of a rectangle of floating-point elements.
 * This class makes it easy to read PNG files and the like from
 * files.
 * Note that colours in the range [0.0, 1.0] are mapped to the integer
 * range [0, 255] when writing and reading PNG files.
 */
class Image {
public:
  Image(); ///< Construct an empty image
  Image(int width, int height, int depth); ///< Construct a black
                                             ///image at the given width/height/depth
  Image(const Image& other); ///< Copy an image

  ~Image();

  Image& operator=(const Image& other); ///< Copy the data from
                                            ///one image to another

  int width() const; ///< Determine the width of the image
  int height() const; ///< Determine the height of the image
  int elements() const; ///< Determine the depth (doubles per pixel) of
                        ///the image

  double operator()(int x, int y, int i) const; ///< Retrieve a
                                               ///particular component
                                               ///from the image.
  double& operator()(int x, int y, int i);  ///< Retrieve a
                                               ///particular component
                                               ///from the image.

  double& pix(int x, int y, int i) { return (*this)(x, y, i); }

  void setColour(int x, int y, Colour &colour);


  bool loadPng(const std::string& filename); ///< Load a PNG file into
                                             /// this image.

  bool savePng(const std::string& filename); ///< Save this image into
                                             ///  the given PNG file
  
  const double* data() const;
  double* data();

  void gammaCorrect(double gamma);
  void normalize();

  Colour getColour(int x, int y);
  Colour getColour(Point2D uv);
  Colour bilinearGetColour(Point2D uv);

  double luminance(int x, int y);
  double maxLuminance();
  double L(int x, int y, double a, double LwBar);

  void ReinhardToneMap(double LwhiteFactor, double a);
  void computeLuminances(Image &L);
  
private:
  int m_width, m_height;
  int m_elements;
  double* m_data;
};

#endif
