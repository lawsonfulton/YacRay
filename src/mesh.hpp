#ifndef CS488_MESH_HPP
#define CS488_MESH_HPP

#include <vector>
#include <iosfwd>
#include "primitive.hpp"
#include "algebra.hpp"

class BoundingBox : public Primitive {
public:
  BoundingBox(const Point3D &minP, const Point3D &maxP);
  BoundingBox();

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);
  
private:
  Point3D mMinP;
  Point3D mMaxP;

  Point3D mExtents;
};

// A polygonal mesh.
class Mesh : public Primitive {
public:
  Mesh(const std::vector<Point3D>& verts,
       const std::vector< std::vector<int> >& faces);

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);
  
  typedef std::vector<int> Face;
  
private:
  bool pointInConvexPolygon(const Point3D &point, const Face &face, const Vector3D &normal);
  
  std::vector<Point3D> m_verts;
  std::vector<Vector3D> m_normals;
  std::vector<Face> m_faces;

  BoundingBox mBoundingBox;

  friend std::ostream& operator<<(std::ostream& out, const Mesh& mesh);
};


class TriMesh : public Primitive {
public:
  TriMesh(const char* obj_path);

  virtual bool rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv);

  typedef std::vector<int> Face;

private:
  bool triangleIntersection(const Point3D &V1, const Point3D &V2, const Point3D &V3, const Ray &ray, double &t);

  std::vector<Point3D> mVerts;
  std::vector<Vector3D> mNormals;
  std::vector<Point2D> mTexCoords;
  std::vector<int> mIndices; //Grouped in threes
  int mNumFaces;

  bool mHasVertNormals;

  BoundingBox mBoundingBox;
};


#endif



