// //---------------------------------------------------------------------------
// //
// // CS488 -- Introduction to Computer Graphics
// //
// // algebra.hpp/algebra.cpp
// //
// // Classes and functions for manipulating points, vectors, matrices, 
// // and colours.  You probably won't need to modify anything in these
// // two files.
// //
// // University of Waterloo Computer Graphics Lab / 2003
// //
// //---------------------------------------------------------------------------

#include "algebra.hpp"
#include <random>

string toString(const QVector4D &vec) {
    ostringstream oss;
    oss << "<" << vec.x() << ", " << vec.y() << ", " << vec.z() << ", " << vec.w() << ">";
    return oss.str();
}

string toString(const QVector3D &vec) {
    ostringstream oss;
    oss << "<" << vec.x() << ", " << vec.y() << ", " << vec.z() << ">";
    return oss.str();
}

string toString(const QVector2D &vec) {
    ostringstream oss;
    oss << "<" << vec.x() << ", " << vec.y() << ">";
    return oss.str();
}

string toString(const Matrix4x4 &mat) {
    ostringstream oss;
    oss << "[ " << toString(mat.row(0)) << endl;
    oss << toString(mat.row(1)) << endl;
    oss << toString(mat.row(2)) << endl;
    oss << toString(mat.row(3)) << " ] " << endl;

    return oss.str();
}

std::ostream& operator <<(std::ostream& os, const QVector3D& v)
{
    return os << toString(v);
}

/*      \|/
 * --------------
 *
 */
Vector3D reflect(const Vector3D &dir, const Vector3D &normal) {
  return 2.0 * dot(dir, normal) * normal - dir;
}

//t is parameter in [0,1] and 
//f(a,b,t) -> a + t(b-a)
template<typename T>
T lin_interpolate(const T &a, const T &b, const double &t) {
  return a + t * (b - a);
}


// Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
// http://realtimecollisiondetection.net/
void getBarycentricCoordinates(const Point3D &p,
                               const Point3D &a,
                               const Point3D &b,
                               const Point3D &c,
                               double &u, double &v, double &w)
{
    Vector3D v0 = b - a, v1 = c - a, v2 = p - a;
    double d00 = dot(v0, v0);
    double d01 = dot(v0, v1);
    double d11 = dot(v1, v1);
    double d20 = dot(v2, v0);
    double d21 = dot(v2, v1);
    double denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

template<typename T>
T barycentricInterpolate(const Point3D &v0, 
                         const Point3D &v1,
                         const Point3D &v2,
                         const Point3D &point,
                         const T &i0, const T &i1, const T &i2) {
    double l0, l1, l2;
    getBarycentricCoordinates(point, v0, v1, v2, l0, l1, l2);

    return l0 * i0 + l1 * i1 + l2 * i2;
}


template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

Vector3D vectorMax(const Vector3D &a, const Vector3D &b) {
  return Vector3D(max(a.x(),b.x()), max(a.y(),b.y()), max(a.z(),b.z()));
}

Vector3D vectorMin(const Vector3D &a, const Vector3D &b) {
  return Vector3D(min(a.x(),b.x()), min(a.y(),b.y()), min(a.z(),b.z()));
}


Vector3D uniformRandomUnitVec() {
    double z = uniformUnitVal();
    double rxy = sqrt(1 - z*z);
    double phi = uniformAngleVal();
    double x = rxy * cos(phi);
    double y = rxy * sin(phi);

    return Vector3D(x,y,z);
}

Vector3D uniformRandomHemisphereUnitVec() {
    Vector3D vec = uniformRandomUnitVec();
    return Vector3D(abs(vec.x()),abs(vec.y()),abs(vec.z()));
}


//Dummy instantiations of templates to avoid linker errors
template QVector3D barycentricInterpolate<QVector3D>(QVector3D const&, QVector3D const&, QVector3D const&, QVector3D const&, QVector3D const&, QVector3D const&, QVector3D const&);
template Point2D barycentricInterpolate<Point2D>(QVector3D const&, QVector3D const&, QVector3D const&, QVector3D const&, Point2D const&, Point2D const&, Point2D const&);

