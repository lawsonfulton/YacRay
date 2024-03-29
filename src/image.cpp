#include "image.hpp"
#include <string>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <png.h>
#include <zlib.h>
#include <sstream>

Image::Image()
  : m_width(0), m_height(0), m_elements(0), m_data(0)
{
}

Image::Image(int width, int height, int elements)
  : m_width(width), m_height(height), m_elements(elements),
    m_data(new double[m_width * m_height * m_elements])
{
}

Image::Image(const Image& other)
  : m_width(other.m_width), m_height(other.m_height), m_elements(other.m_elements),
    m_data(other.m_data ? new double[m_width * m_height * m_elements] : 0)
{
  if (m_data) {
    std::memcpy(m_data, other.m_data,
                m_width * m_height * m_elements * sizeof(double));
  }
}

Image::~Image()
{
  delete [] m_data;
}

Image& Image::operator=(const Image& other)
{
  delete [] m_data;
  
  m_width = other.m_width;
  m_height = other.m_height;
  m_elements = other.m_elements;
  m_data = (other.m_data ? new double[m_width * m_height * m_elements] : 0);

  if (m_data) {
    std::memcpy(m_data,
                other.m_data,
                m_width * m_height * m_elements * sizeof(double));
  }
  
  return *this;
}

int Image::width() const
{
  return m_width;
}

int Image::height() const
{
  return m_height;
}

int Image::elements() const
{
  return m_elements;
}

double Image::operator()(int x, int y, int i) const
{
  return m_data[m_elements * (m_width * y + x) + i];
}

double& Image::operator()(int x, int y, int i)
{
  return m_data[m_elements * (m_width * y + x) + i];
}

bool Image::savePng(const std::string& filename)
{
  FILE* fout = std::fopen(filename.c_str(), "wb");
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  png_infop info_ptr = png_create_info_struct(png_ptr);
  //setjmp(png_ptr->jmpbuf); //TODO might need to keep this

  /* Setup PNG I/O */
  png_init_io(png_ptr, fout);
	 
  /* Optionally setup a callback to indicate when a row has been
   * written. */  

  /* Setup filtering. Use Paeth filtering */
  png_set_filter(png_ptr, 0, PNG_FILTER_PAETH);

  /* Setup compression level. */
  png_set_compression_level(png_ptr, Z_BEST_COMPRESSION);

  /* Setup PNG header information and write it to the file */

  int color_type;
  switch (m_elements) {
  case 1:
    color_type = PNG_COLOR_TYPE_GRAY;
    break;
  case 2:
    color_type = PNG_COLOR_TYPE_GRAY_ALPHA;
    break;
  case 3:
    color_type = PNG_COLOR_TYPE_RGB;
    break;
  case 4:
    color_type = PNG_COLOR_TYPE_RGBA;
    break;
  default:
    return false;
  }
   
  png_set_IHDR(png_ptr, info_ptr,
               m_width, m_height,
               8, 
               color_type,
               PNG_INTERLACE_NONE, 
               PNG_COMPRESSION_TYPE_DEFAULT, 
               PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png_ptr, info_ptr); 
    
  // Actual writing
  png_byte* tempLine = new png_byte[m_width * m_elements];
  
  for(int i=0;i<m_height;i++){
    for(int j=0;j<m_width;j++){
      for(int k = 0;k<m_elements;k++) {
        // Clamp the value
        double value = std::min(1.0, std::max(0.0, (*this)(j, i, k)));
        
        // Write it out
        tempLine[m_elements*j+k] = static_cast<png_byte>(value*255.0); 
      }
    }
    png_write_row(png_ptr, tempLine);
  }
  
  delete [] tempLine;
  
  // closing and freeing the structs
  png_write_end(png_ptr, info_ptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
  fclose(fout);

  return true;
}

bool Image::loadPng(const std::string& filename)
{
  // check that the file is a png file
  png_byte buf[8];

  FILE* in = std::fopen(filename.c_str(), "rb");

  if (!in) return false;
  
  for (int i = 0; i < 8; i++) {
    if (!(buf[i] = fgetc(in))) return false;
  }
  if (png_sig_cmp(buf, 0, 8)) return false;

  png_structp png_ptr =
    png_create_read_struct(PNG_LIBPNG_VER_STRING,
			   0, 0, 0);
  if (!png_ptr) return false;

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_read_struct(&png_ptr, 0, 0);
    return false;
  }

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_read_struct(&png_ptr, 0, 0);
    return false;
  }
  
  //  png_set_read_fn(png_ptr, reinterpret_cast<void*>(&in), my_istream_read_data);
  png_init_io(png_ptr, in);
  
  png_set_sig_bytes(png_ptr, 8);

  png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, 0);

  int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
  if (bit_depth % 8) {
    png_destroy_read_struct(&png_ptr, 0, 0);
    std::ostringstream os;
    os << "Invalid bit elements " << bit_depth;
    return false;
  }

  int colour_type = png_get_color_type(png_ptr, info_ptr);

  if (colour_type == PNG_COLOR_TYPE_PALETTE) {
    png_set_palette_to_rgb(png_ptr);
    colour_type = PNG_COLOR_TYPE_RGB;
  }

  if (colour_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) {
    png_set_expand_gray_1_2_4_to_8(png_ptr);
  }

  if (colour_type != PNG_COLOR_TYPE_RGB
      && colour_type != PNG_COLOR_TYPE_GRAY
      && colour_type != PNG_COLOR_TYPE_RGBA) {
    png_destroy_read_struct(&png_ptr, 0, 0);
    return false;
  }

  delete [] m_data;

  m_width = png_get_image_width(png_ptr, info_ptr);
  m_height = png_get_image_height(png_ptr, info_ptr);
  switch (colour_type) {
  case PNG_COLOR_TYPE_RGB:
    m_elements = 3;
    break;
  case PNG_COLOR_TYPE_RGBA:
    m_elements = 4;
    break;
  default:
    m_elements = 1;
    break;
  }
  
  png_bytep* row_pointers = png_get_rows(png_ptr, info_ptr);

  m_data = new double[m_width * m_height * m_elements];

  for (int y = 0; y < m_height; y++) {
    for (int x = 0; x < m_width; x++) {
      for (int i = 0; i < m_elements; i++) {
	png_byte *row = row_pointers[y];
	int index = m_elements * (y * m_width + x) + i;
	
        long element = 0;
        for (int j = bit_depth/8 - 1; j >= 0; j--) {
          element <<= 8;
          element += row[(x * m_elements + i) * bit_depth/8 + j];
        }
        
	m_data[index] = element / static_cast<double>((1 << bit_depth) - 1);
      }
    }
  }

  png_destroy_read_struct(&png_ptr, &info_ptr, 0);

  return true;
}

const double* Image::data() const
{
  return m_data;
}

double* Image::data()
{
  return m_data;
}

void Image::setColour(int x, int y, Colour &colour) {
  (*this)(x, y, 0) = colour.R();
  (*this)(x, y, 1) = colour.G();
  (*this)(x, y, 2) = colour.B();
}

void Image::gammaCorrect(double gamma) {
  //g'(g) = w_max * ((g - w_min) / (w_max - w_min)) ^ gamma + w_min

  for(int component = 0; component < 3; component++) {
    double wMax = 0.0;
    double wMin = DBL_INF;

    //Compute max and min
    for(int y = 0; y < m_height; y++) {
      for(int x = 0; x < m_width; x++) {
        double g = (*this)(x,y,component);

        wMax = max(g, wMax);
        wMin = min(g, wMin);
      }
    }

    //Correct
    for(int y = 0; y < m_height; y++) {
      for(int x = 0; x < m_width; x++) {
        double &g = (*this)(x,y,component);
        g = wMax * pow(((g - wMin) / (wMax - wMin)), gamma) + wMin;
        g = g / wMax;
      }
    }
  }
}

void Image::normalize() {
  double wMax = 0.0;
  for(int component = 0; component < 3; component++) {
    //Compute max and min
    for(int y = 0; y < m_height; y++) {
      for(int x = 0; x < m_width; x++) {
        double g = (*this)(x,y,component);

        wMax = max(g, wMax);
      }
    }
  }

    //Correct
  for(int component = 0; component < 3; component++) {
    for(int y = 0; y < m_height; y++) {
      for(int x = 0; x < m_width; x++) {
        double &g = (*this)(x,y,component);
        g = g / wMax;
      }
    }
  }
}

Colour Image::getColour(int x, int y) {
  if(x >= width()) { 
    x = x % width(); 
  }
  else if(x < 0) { 
    x = (-x) % width();
    x = width() - x;
  }

  if(y >= height()) { 
    y = y % height(); 
  }
  else if(y < 0) { 
    y = (-y) % height();
    y = height() - y;
  }

  return Colour((*this)(x,y,0),(*this)(x,y,1),(*this)(x,y,2));
}

Colour Image::getColour(Point2D uv) {
  int x = floor(uv.x() * width() + 0.5);
  int y = floor(uv.y() * height() + 0.5);

  return getColour(x, y);
}

Colour Image::bilinearGetColour(Point2D uv) {
  //Source: http://fastcpp.blogspot.ca/2011/06/bilinear-pixel-interpolation-using-sse.html
  double x = uv.x() * width();
  double y = uv.y() * height();

  int px = (int)floor(x);
  int py = (int)floor(y);


  // load the four neighboring pixels
  Colour p1 = getColour(px + 0, py + 0);
  Colour p2 = getColour(px + 1, py + 0);
  Colour p3 = getColour(px + 0, py + 1);
  Colour p4 = getColour(px + 1, py + 1);

  // Calculate the weights for each pixel
  double fx = x - (double)px;
  double fy = y - (double)py;
  double fx1 = 1.0 - (double)fx;
  double fy1 = 1.0 - (double)fy;

  double w1 = fx1 * fy1;
  double w2 = fx  * fy1;
  double w3 = fx1 * fy;
  double w4 = fx  * fy;

  Colour out = p1 * w1 + p2 * w2 + p3 * w3 + p4 * w4;

 if(out.R() > 1.0) {
  cout << "Colour error" << endl;
  cout << p1 << " " << p2 << " " << p3 << " " << p4 << endl;
  cout << w1 << " " << w2 << " " << w3 << " " << w4 << endl;
}
 return out;
}

double Image::luminance(int x, int y) {
  return (0.2126 * pix(x,y,0) + 0.7152 * pix(x,y,1) + 0.0722 * pix(x,y,2));
}

double Image::maxLuminance() {
  double wMax = 0.0;

  //Compute max and min
  for(int y = 0; y < m_height; y++) {
    for(int x = 0; x < m_width; x++) {
      double g = pix(x,y,0);

      wMax = max(g, wMax);
    }
  }

  return wMax;
}

double computeLwBar(Image &Lw) {
  double delta = 0.00001; //TODO how much?
  double sum = 0.0; //Might get error accum

  for(int y = 0; y < Lw.height(); y++) {
    for(int x = 0; x < Lw.width(); x++) {
      sum += log(delta + Lw(x,y,0));
    }
  }

  return exp(sum / (Lw.width() * Lw.height()));
} 

void simpleMap(Image &Lw, double a, double LwBar) {
  for(int y = 0; y < Lw.height(); y++) {
    for(int x = 0; x < Lw.width(); x++) {
      double lxy = Lw.L(x,y,a,LwBar);

      Lw(x,y,0) = lxy / (1.0 + lxy);
    }
  }
}

//Yes this is a terrible ugly hack
//Use precomputed LwBar and a to compute image luminance
double Image::L(int x, int y, double a, double LwBar) {
  return a / LwBar * pix(x,y,0);
}

void Image::ReinhardToneMap(double LwhiteFactor, double a) {
  //Map into luminances
  Image Lw(width(), height(), 1);
  computeLuminances(Lw);

  double LwBar = computeLwBar(Lw);
  double Lwhite = Lw.maxLuminance() * LwhiteFactor;

  for(int y = 0; y < height(); y++) {
    for(int x = 0; x < width(); x++) {
      double lxy = Lw.L(x,y,a,LwBar);

      Lw(x,y,0) = (lxy * (1.0 + lxy/pow(Lwhite,2)))/ (1.0 + lxy);
    }
  }

  //Remap to RGB
  for(int y = 0; y < height(); y++) {
    for(int x = 0; x < width(); x++) {
      double scale = Lw(x,y,0)/luminance(x, y);

      for(int i = 0; i < 3; i++) {
        pix(x,y,i) *= scale;
      }
    }
  }

}


void Image::computeLuminances(Image &L) {
  for(int y = 0; y < L.height(); y++) {
    for(int x = 0; x < L.width(); x++) {
      L(x,y,0) = luminance(x,y);
    }
  }
}
































