#include "mesh.hpp"
#include "ray.hpp" 
#include "options.hpp"
#include "algebra.hpp"
#include "tiny_obj_loader.h"

#include <iostream>
#include <chrono>

using namespace std::chrono;

Point3D Tri::midpoint(const TriMesh *m) {
    Point3D midpt;
    for(int i = 0; i < 3; i++) {
        midpt = midpt + m->mVerts[p[i]];
    }

    return midpt / 3.0;
}

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

bool Mesh::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
  bool hitBbox = mBoundingBox.rayIntersection(ray, t, normal, point, uv);
  
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

BoundingBox::BoundingBox(vector<Tri> &faces, const TriMesh *triMesh) {
    if(faces.size() == 0) return;

    mMaxP = Point3D(-DBL_INF, -DBL_INF, -DBL_INF);
    mMinP = Point3D(DBL_INF, DBL_INF, DBL_INF);

    for(int i = 0; i < (int)faces.size(); i++) {
        for(int v = 0; v < 3; v++) {
            mMaxP = vectorMax(mMaxP, triMesh->mVerts[faces[i][v]]);
            mMinP = vectorMin(mMinP, triMesh->mVerts[faces[i][v]]);
        }
    }

   //cout << mMinP << " " << mMaxP << endl;
    //mMinP = mMinP - EPSILON_VEC;
    //mMaxP = mMaxP + EPSILON_VEC;

    mExtents = (mMaxP - mMinP) / 2.0;
}

BoundingBox::BoundingBox() {
}


int BoundingBox::longestAxis() {
    int axis;
    double longest = -DBL_INF;

    for(int i = 0; i < 3; i++) {
      double len = mMaxP[i] - mMinP[i];
      if(len > longest) {
        longest = len;
        axis = i;
      }
    }
    return axis;
}

bool BoundingBox::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
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

bool BoundingBox::hit(const Ray &ray) {
    //cout << mMinP << " " << mMaxP << endl;
    //exit(1);
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
    return true;
  }
  else {
    return false;
  }
}

//*******************

TriMesh::TriMesh(const char* obj_path) {
    //Load the obj file using 3rd party library
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err = tinyobj::LoadObj(shapes, materials, obj_path);

    tinyobj::mesh_t &mesh = shapes[0].mesh;

    mVerts.resize(mesh.positions.size()/3);
    mNormals.resize(mesh.normals.size()/3);
    mIndices.resize(mesh.indices.size());
    mTexCoords.resize(mesh.texcoords.size()/2);
    mNumFaces = (int)mIndices.size()/3;

    for(int i = 0; i < (int)mesh.positions.size()/3; i++) {
        mVerts[i] = Point3D(mesh.positions[3 * i + 0],
                            mesh.positions[3 * i + 1],
                            mesh.positions[3 * i + 2]);
    }

    for(int i = 0; i < (int)mesh.normals.size()/3; i++) {
        mNormals[i] = Vector3D(mesh.normals[3 * i + 0],
                               mesh.normals[3 * i + 1],
                               mesh.normals[3 * i + 2]);
    }

    for(int i = 0; i < (int)mesh.indices.size(); i++) {
        mIndices[i] = (int)mesh.indices[i];
    }

    for(int i = 0; i < (int)mesh.texcoords.size()/2; i++) {
        mTexCoords[i] = Point2D(mesh.texcoords[2 * i + 0],
                               mesh.texcoords[2 * i + 1]);
    }

    //Default uv mapping sphere projection
    if(mTexCoords.size() == 0) {
        mTexCoords.resize(mVerts.size());
        for(int i = 0; i < (int)mVerts.size(); i++) {
            Point3D &point = mVerts[i];
            double u = 0.5 + atan2(point.z(), -point.x()) / (2.0 * M_PI);
            double v = 0.5 - asin(point.y()) / M_PI;
            mTexCoords[i] = Point2D(u,v);
        }
    }

    //Calc normals if we didn't get any
    if(mNormals.size() == 0) {
        mHasVertNormals = false;

        for(int i = 0; i < mNumFaces; i++) {
            int *face = &mIndices[i * 3];
            Vector3D planeNormal = cross(mVerts[face[0]] - mVerts[face[2]], mVerts[face[1]] - mVerts[face[2]]).normalized();
            mNormals.push_back(planeNormal);
        }

    } else {
        mHasVertNormals = true;
    }

    //Set up bounding box
    Point3D maxP = Point3D(-DBL_INF, -DBL_INF, -DBL_INF);
    Point3D minP = Point3D(DBL_INF, DBL_INF, DBL_INF);

    for(int i = 0; i < (int)mVerts.size(); i++) {
        maxP = vectorMax(maxP, mVerts[i]);
        minP = vectorMin(minP, mVerts[i]);
    }

    mBoundingBox = BoundingBox(minP, maxP);

    mFaces.reserve(mNumFaces);
    for(int i = 0; i < mNumFaces; i++) {
        int *face = &mIndices[i * 3];
        Tri t;
        t.p = face;
        mFaces.push_back(t);
    }

    cout << "Building KdTree..." << endl;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    mKdTree = new KdNode(this);//TODO fix this
    mKdTree = mKdTree->build(mFaces, 0);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    cout << "Finished in: " << duration/1000000.0 << endl;
}

TriMesh::~TriMesh() {
    delete mKdTree;
}

bool TriMesh::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
    bool hitBbox = mBoundingBox.rayIntersection(ray, t, normal, point, uv);

    #if DRAW_BOUNDING_BOXES
    return hitBbox;
    #endif

    if(hitBbox) {
        double minT = DBL_INF;
        bool intersects = false;

        // for(int i = 0; i < mNumFaces; i++) {
        //     double newT = DBL_INF;
        //     int *face = &mIndices[i * 3];

        //     const Point3D &v0 = mVerts[face[0]];
        //     const Point3D &v1 = mVerts[face[1]];
        //     const Point3D &v2 = mVerts[face[2]];

        //     bool hit = triangleIntersection(v0, v1, v2, ray, newT);

        //     if(hit && newT > MIN_INTERSECT_DIST) {
        //         if(newT < minT) {
        //             minT = newT;
        //             point = ray.getPoint(newT);
        //             intersects = true;
                    
        //             if(mHasVertNormals) { //TODO move this out of main loop?
        //                 const Vector3D &n0 = mNormals[face[0]];
        //                 const Vector3D &n1 = mNormals[face[1]];
        //                 const Vector3D &n2 = mNormals[face[2]];

        //                 normal = barycentricInterpolate(v0, v1, v2, point, n0, n1, n2);//barycentricInterpolate(v2, v1, v0, point, n2, n1, n0);
                        
        //                 const Point2D &uv0 = mTexCoords[face[0]];
        //                 const Point2D &uv1 = mTexCoords[face[1]];
        //                 const Point2D &uv2 = mTexCoords[face[2]];
        //                 uv = barycentricInterpolate(v0, v1, v2, point, uv0, uv1, uv2);
        //             }
        //             else {
        //                 normal = mNormals[i];
        //             }
        //         }
        //     }
        // }

        Tri face;
        intersects = mKdTree->rayIntersection(ray, t, face);

        if(intersects) {
            // cout << "hit!" << endl;
            // cout << t << endl;
            point = ray.getPoint(t);
            // cout << face[0] << " " << face[1] << " " <<face[2] << endl;
            if(mHasVertNormals) { //TODO move this out of main loop?
                const Point3D &v0 = mVerts[face[0]];
                const Point3D &v1 = mVerts[face[1]];
                const Point3D &v2 = mVerts[face[2]];
                const Vector3D &n0 = mNormals[face[0]];
                const Vector3D &n1 = mNormals[face[1]];
                const Vector3D &n2 = mNormals[face[2]];

                normal = barycentricInterpolate(v0, v1, v2, point, n0, n1, n2);//barycentricInterpolate(v2, v1, v0, point, n2, n1, n0);
                
                const Point2D &uv0 = mTexCoords[face[0]];
                const Point2D &uv1 = mTexCoords[face[1]];
                const Point2D &uv2 = mTexCoords[face[2]];
                uv = barycentricInterpolate(v0, v1, v2, point, uv0, uv1, uv2);
            }
            else { 
                // cout << "need to store normals differently" << endl;
                //normal = mNormals[i];
            }
            // cout << "ok" << endl;
        }

        return intersects;
    }
    else {
        return false;
    }
}

#define TRI_EPSILON 0.000001
//http://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool TriMesh::triangleIntersection( const Point3D &V1,  // Triangle vertices
                                   const Point3D &V2,
                                   const Point3D &V3,
                                   const Ray &ray,
                                   double &t) const
{
  Vector3D e1, e2;  //Edge1, Edge2
  Vector3D P, Q, T;
  double det, inv_det, u, v; //TODO - should be double?
  double newT;
 
  //Find vectors for two edges sharing V1
  e1 = V2 - V1;
  e2 = V3 - V1;

  //Begin calculating determinant - also used to calculate u parameter
  P = cross(ray.direction(), e2);
  //if determinant is near zero, ray lies in plane of triangle
  det = dot(e1, P);
  //NOT CULLING
  if(det > -TRI_EPSILON && det < TRI_EPSILON) return false;
  inv_det = 1.0 / det;
 
  //calculate distance from V1 to ray origin
  T = ray.origin() - V1;
 
  //Calculate u parameter and test bound
  u = dot(T, P) * inv_det;

  //The intersection lies outside of the triangle
  if(u < 0.0 || u > 1.0) return false;
 
  //Prepare to test v parameter
  Q = cross(T, e1);
 
  //Calculate V parameter and test bound
  v = dot(ray.direction(), Q) * inv_det;
  //The intersection lies outside of the triangle
  if(v < 0.0 || u + v  > 1.0) return false;
 
  newT = dot(e2, Q) * inv_det;
 
  if(newT > TRI_EPSILON) { //ray intersection
    t = newT;
    return true;
  }
 
  // No hit, no win
  return false;
}


KdNode::KdNode(const TriMesh *triMesh) {
    mTriMesh = triMesh;
}

KdNode::~KdNode() {
    delete mLeft;
    delete mRight;
}

//153s on dragon before
KdNode* KdNode::build(vector<Tri> &faces, int depth) const {
    KdNode *node = new KdNode(mTriMesh);
    node->mFaces = faces;
    node->mLeft = NULL;
    node->mRight = NULL;
    node->mBbox = BoundingBox(faces, mTriMesh);

    //cout << depth << endl;
    if(faces.size() == 0 || faces.size() == 1) {
        //cout << "hello 0" << endl;
        return node;
    } 

    // if(faces.size() == 1) {
    //     //cout << "hello 1" << endl;
    //     // node->mLeft = new KdNode(mTriMesh);
    //     // node->mRight = new KdNode(mTriMesh);
    //     node->mLeft->mFaces = vector<Tri>();
    //     node->mRight->mFaces = vector<Tri>();
    //     return node;
    // }

    Point3D midpt;
    for(int i = 0; i < (int)faces.size(); i++) {
        midpt = midpt + faces[i].midpoint(mTriMesh);
    }
    midpt = midpt / faces.size();
    //cout << midpt << " " << faces.size() << endl;

    vector<Tri> leftFaces;
    vector<Tri> rightFaces;
    int axis = node->mBbox.longestAxis();
    //cout << "axis: " << axis << endl;
    for(int i = 0; i < (int)faces.size(); i++) {
        //cout << mTriMesh->mVerts[faces[i][0]] << endl;
        //cout << midpt[axis] << " >= " << faces[i].midpoint(mTriMesh)[axis] << " : " << (midpt[axis] >= faces[i].midpoint(mTriMesh)[axis]) << endl;
        if(midpt[axis] >= faces[i].midpoint(mTriMesh)[axis]) {
            rightFaces.push_back(faces[i]);
        } else {
            leftFaces.push_back(faces[i]);
        }
    }

    // cout << " right: " << rightFaces.size() << endl;
    // cout << " left: " << leftFaces.size() << endl;

    if(leftFaces.size() == 0 && rightFaces.size() > 0) {
        leftFaces = rightFaces;
    }
    if(rightFaces.size() == 0 && leftFaces.size() > 0) {
        rightFaces = leftFaces;
    }

    int matches = 0;
    for(int i = 0; i < (int)leftFaces.size(); i++) {
        for(int j = 0; j < (int)rightFaces.size(); j++) {
            if(leftFaces[i].p == rightFaces[j].p) {
                matches++;
            }
        }
    }
    // cout << "matches: " << matches << " right: " << rightFaces.size() << endl;
    // cout << "matches: " << matches << " left: " << leftFaces.size() << endl;

    if(matches/(double)leftFaces.size() < 0.5 && matches/(double)rightFaces.size() < 0.5) {
        node->mLeft = build(leftFaces, depth + 1);
        node->mRight = build(rightFaces, depth + 1);
    }

    return node;
}

bool KdNode::rayIntersection(const Ray &ray, double &t, Tri &face) {
    double leftT, rightT;
    bool hitLeft = false, hitRight = false;
    Tri leftFace, rightFace;

    if(mBbox.hit(ray)) {
        //cout << "hitbox" << endl;
        if(mLeft) {
            //cout << "before l" << endl;
            hitLeft = mLeft->rayIntersection(ray, leftT, leftFace);
            //cout << "after l" << endl;
            hitRight = mRight->rayIntersection(ray, rightT, rightFace);
            //cout << "after" << endl;
            if(hitLeft && hitRight) {
                if(leftT <= rightT) {
                    t = leftT;
                    face = leftFace;
                }
                else if(rightT < leftT) {
                    t = rightT;
                    face = rightFace;
                }

                return true;
            }
            else if(hitLeft) {
                t = leftT;
                face = leftFace;
                return true;
            }
            else if(hitRight) {
                t = rightT;
                face = rightFace;
                return true;
            }

            return false;
        }
        else {
            double closestT = DBL_INF;
            Tri closestF;

            //cout << "before " << endl;
            bool hitTri = false;
            for(int i = 0; i < (int)mFaces.size(); i++) {
                double thisT;
                const Point3D &v0 = mTriMesh->mVerts[mFaces[i][0]];
                const Point3D &v1 = mTriMesh->mVerts[mFaces[i][1]];
                const Point3D &v2 = mTriMesh->mVerts[mFaces[i][2]];

                bool hit = mTriMesh->triangleIntersection(v0, v1, v2, ray, thisT);
                if(hit) {
                    hitTri = true;
                    if(thisT < closestT) {
                         // cout << "goooood" << endl;
                    // cout << thisT << endl; 
                        closestT = thisT; 
                        closestF = mFaces[i];
                    }
                }
            }
            //cout << "after" << endl;

            if(hitTri) {
                //cout << "t: " << closestT << endl;
                t = closestT;
                face = closestF;
            }

            return hitTri;
        }
    } 
    else {
        return false;
    }
}















