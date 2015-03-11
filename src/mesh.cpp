#include "mesh.hpp"
#include "ray.hpp" 
#include "options.hpp"

#include <iostream>

Mesh::Mesh(const std::vector<Point3D>& verts,
           const std::vector< std::vector<int> >& faces)
  : m_verts(verts),
    m_faces(faces)
{
  Point3D maxP = Point3D(-DBL_INF, -DBL_INF, -DBL_INF);
  Point3D minP = Point3D(DBL_INF, DBL_INF, DBL_INF);

  for(int i = 0; i < (int)verts.size(); i++) {
    maxP = vectorMax(maxP, verts[i]);
    minP = vectorMin(minP, verts[i]);
  }

  mBoundingBox = BoundingBox(minP, maxP);

  for(int i = 0; i < (int)m_faces.size(); i++) {
      Face &face = m_faces[i];
      Vector3D planeNormal = cross(m_verts[face[0]] - m_verts[face[2]], m_verts[face[1]] - m_verts[face[2]]).normalized();
      m_normals.push_back(planeNormal);
  }
}

std::ostream& operator<<(std::ostream& out, const Mesh& mesh)
{
  std::cerr << "mesh({";
  for (std::vector<Point3D>::const_iterator I = mesh.m_verts.begin(); I != mesh.m_verts.end(); ++I) {
    if (I != mesh.m_verts.begin()) std::cerr << ",\n      ";
    std::cerr << toString(*I);
  }
  std::cerr << "},\n\n     {";
  
  for (std::vector<Mesh::Face>::const_iterator I = mesh.m_faces.begin(); I != mesh.m_faces.end(); ++I) {
    if (I != mesh.m_faces.begin()) std::cerr << ",\n      ";
    std::cerr << "[";
    for (Mesh::Face::const_iterator J = I->begin(); J != I->end(); ++J) {
      if (J != I->begin()) std::cerr << ", ";
      std::cerr << *J;
    }
    std::cerr << "]";
  }
  std::cerr << "});" << std::endl;
  return out;
}

bool Mesh::pointInConvexPolygon(const Point3D &point, const Face &face, const Vector3D &normal) {
  for(int j = (int)face.size() - 1; j >= 0; j--) {
    int d = j-1;
    if(j-1 == -1) d = (int)face.size() - 1;

    Vector3D hsNormal = -cross(normal, m_verts[face[d]] - m_verts[face[j]]); 
    if(dot(point - m_verts[face[j]], hsNormal) < 0.0) {//not in halfspace -- reverse?
      return false;
    }
  }

  return true;
} 

bool Mesh::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
  bool hitBbox = mBoundingBox.rayIntersection(ray, t, normal, point);
  
  #if DRAW_BOUNDING_BOXES
  return hitBbox;
  #endif

  if(hitBbox) {
    double minT = DBL_INF;
    Point3D minP;
    Vector3D minN;
    bool intersects = false;

    for(int i = 0; i < (int)m_faces.size(); i++) {
      Face &face = m_faces[i];
      //Vector3D planeNormal = cross(m_verts[face[0]] - m_verts[face[2]], m_verts[face[1]] - m_verts[face[2]]).normalized();

      //Find point on plane
      double lDn = dot(ray.direction(), m_normals[i]);
      double t = dot((m_verts[face[0]] - ray.origin()), m_normals[i]) / lDn;
      Point3D planePoint = ray.getPoint(t);

      //If it hits the plane
      if(t > MIN_INTERSECT_DIST) {
        bool inPoly = pointInConvexPolygon(planePoint, face, m_normals[i]);

        if(inPoly) {
          intersects = true;

          if(t < minT) {
            minT = t;
            minP = planePoint;
            minN = m_normals[i];
          }
        }
      }
    }

    t = minT;
    normal = minN;
    point = minP;

    return intersects;
  }
  else {
    return false;
  }
}

BoundingBox::BoundingBox(const Point3D &minP, const Point3D &maxP) {
  mMinP = minP - EPSILON_VEC;
  mMaxP = maxP + EPSILON_VEC;

  mExtents = (maxP - minP) / 2.0;
}

BoundingBox::BoundingBox() {
}

bool BoundingBox::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
  //Alg from http://tavianator.com/2011/05/fast-branchless-raybounding-box-intersections/
  Point3D rayOrg = ray.origin();
  Point3D rayDir = ray.direction();

  double tx1 = (mMinP.x() - rayOrg.x())/rayDir.x();
  double tx2 = (mMaxP.x() - rayOrg.x())/rayDir.x();

  double tmin = min(tx1, tx2);
  double tmax = max(tx1, tx2);

  double ty1 = (mMinP.y() - rayOrg.y())/rayDir.y();
  double ty2 = (mMaxP.y() - rayOrg.y())/rayDir.y();

  tmin = max(tmin, min(ty1, ty2));
  tmax = min(tmax, max(ty1, ty2));

  double tz1 = (mMinP.z() - rayOrg.z())/rayDir.z();
  double tz2 = (mMaxP.z() - rayOrg.z())/rayDir.z();

  tmin = max(tmin, min(tz1, tz2));
  tmax = min(tmax, max(tz1, tz2));

  if(tmax >= max(0.0, tmin)) { //intersects
    Point3D centroid = mMinP + mExtents;
    t = tmin;
    point = ray.getPoint(t);

    //Compute normal by finding compennent with biggest magnitude
    Point3D newP = point - centroid;//Need to do this around origin
    float min_distance = DBL_INF;

    for (int i = 0; i < 3; ++i) {
        float distance = abs(mExtents[i] - abs(newP[i]));

        if (distance < min_distance) {
            min_distance = distance;

            normal = Vector3D();
            normal[i] = copysign(1.0, newP[i]);
        }
    }

    return true;
  }
  else {
    return false;
  }
}