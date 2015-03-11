#include "primitive.hpp"
#include "polyroots.hpp"
#include "ray.hpp"

Primitive::~Primitive()
{
}

bool Primitive::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
	return false;
}

Sphere::~Sphere()
{
}

bool Sphere::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
	Vector3D oMc = ray.origin();

	double a = 1.0;
	double b = 2.0 * dot(ray.direction(), oMc);
	double c = oMc.lengthSquared() - 1.0;

	double roots[2];
	int numRoots = quadraticRoots(a, b, c, roots);

	if(numRoots == 0) {
		return false;
	}
	else {
		if(numRoots == 2) { //todo change for box too
			if(roots[0] > MIN_INTERSECT_DIST && roots[1] > MIN_INTERSECT_DIST) {
				t = min(roots[0], roots[1]);
			} else {
				if(roots[0] > MIN_INTERSECT_DIST) {
					t = roots[0];
				} else if(roots[1] > MIN_INTERSECT_DIST) {
					t = roots[1];
				} else {
					return false;
				}
			}
		} else {
			if(roots[0] > MIN_INTERSECT_DIST) {
				t = roots[0];
			} else {
				return false;
			}
		}

		point = ray.getPoint(t);
		normal = point;

		// if(c < 0.0 ) {
		// 	//cout << "w" << endl;
		// 	normal = -normal;
		// }
		

		return true;
	} 
}

Cube::~Cube()
{
}

bool Cube::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
	//Alg from http://tavianator.com/2011/05/fast-branchless-raybounding-box-intersections/
	Point3D rayOrg = ray.origin();
	Point3D rayDir = ray.direction();

	double tx1 = (-rayOrg.x())/rayDir.x();
	double tx2 = (1.0 - rayOrg.x())/rayDir.x();

	double tmin = min(tx1, tx2);
	double tmax = max(tx1, tx2);

	double ty1 = (-rayOrg.y())/rayDir.y();
	double ty2 = (1.0 - rayOrg.y())/rayDir.y();

	tmin = max(tmin, min(ty1, ty2));
	tmax = min(tmax, max(ty1, ty2));

	double tz1 = (-rayOrg.z())/rayDir.z();
	double tz2 = (1.0 - rayOrg.z())/rayDir.z();

	tmin = max(tmin, min(tz1, tz2));
	tmax = min(tmax, max(tz1, tz2));


///TODO - remove this?
	if(tmax >= max(0.0, tmin)) { //intersects

	// 	if(tmin > MIN_INTERSECT_DIST && tmin < tmax) {
	// 	t = tmin;
	// } else if(tmax > MIN_INTERSECT_DIST && tmax < tmin) {
	// 	t = tmax;
	// } else {
	// 	return false;
	// }


	if(tmax > MIN_INTERSECT_DIST && tmin > MIN_INTERSECT_DIST) {
		t = min(tmax, tmin);
	} else {
		if(tmax > MIN_INTERSECT_DIST) {
			t = tmax;
		} else if(tmin > MIN_INTERSECT_DIST) {
			t = tmin;
		} else {
			return false;
		}
	}

//		t = tmin;
		point = ray.getPoint(t);

		//Compute normal by finding compennent with biggest magnitude
		Point3D newP = point - Vector3D(0.5,0.5,0.5);

		//
	    float min_distance = DBL_INF;

	    for (int i = 0; i < 3; ++i) {
	        float distance = abs(0.5 - abs(newP[i]));

	        if (distance < min_distance) {
	            min_distance = distance;

	            normal = Vector3D();
	            normal[i] = copysign(1.0, newP[i]);
	        }
	    }
		//if(dot(normal, ray.direction()) > 0.0) normal = -normal; //TODO

		return true;
	}
	else {
		return false;
	}
}

NonhierSphere::~NonhierSphere()
{
}

//TODO idea -- maybe I hsould only return t?
bool NonhierSphere::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
	Vector3D oMc = ray.origin() - m_pos;

	double a = 1.0;
	double b = 2.0 * dot(ray.direction(), oMc);
	double c = oMc.lengthSquared() - m_radius * m_radius;

	double roots[2];
	int numRoots = quadraticRoots(a, b, c, roots);

	if(numRoots == 0) {
		return false;
	}
	else {
		if(numRoots == 2) { //todo change for box too
			if(roots[0] > MIN_INTERSECT_DIST && roots[1] > MIN_INTERSECT_DIST) {
				t = min(roots[0], roots[1]);
			} else {
				if(roots[0] > MIN_INTERSECT_DIST) {
					t = roots[0];
				} else if(roots[1] > MIN_INTERSECT_DIST) {
					t = roots[1];
				} else {
					return false;
				}
			}
		} else {
			if(roots[0] > MIN_INTERSECT_DIST) {
				t = roots[0];
			} else {
				return false;
			}
		}

		point = ray.getPoint(t);
		normal = point - m_pos;
		normal.normalize();

		//ray started inside?
		// if(c < 0.1 ) {
		// 	//cout << "w" << endl;
		// 	normal = -normal;
		// }

		return true;
	} 
}

NonhierBox::~NonhierBox()
{
}

bool NonhierBox::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
	//Alg from http://tavianator.com/2011/05/fast-branchless-raybounding-box-intersections/
	double halfSize = m_size/2.0;
	Point3D minP = m_pos;
	Point3D maxP = m_pos + Vector3D(m_size,m_size,m_size);
	Point3D rayOrg = ray.origin();
	Point3D rayDir = ray.direction();

	double tx1 = (minP.x() - rayOrg.x())/rayDir.x();
	double tx2 = (maxP.x() - rayOrg.x())/rayDir.x();

	double tmin = min(tx1, tx2);
	double tmax = max(tx1, tx2);

	double ty1 = (minP.y() - rayOrg.y())/rayDir.y();
	double ty2 = (maxP.y() - rayOrg.y())/rayDir.y();

	tmin = max(tmin, min(ty1, ty2));
	tmax = min(tmax, max(ty1, ty2));

	double tz1 = (minP.z() - rayOrg.z())/rayDir.z();
	double tz2 = (maxP.z() - rayOrg.z())/rayDir.z();

	tmin = max(tmin, min(tz1, tz2));
	tmax = min(tmax, max(tz1, tz2));

	if(tmax >= max(0.0, tmin)) { //intersects
		t = tmin;
		point = ray.getPoint(t);

		//Compute normal by finding compennent with biggest magnitude
		Point3D newP = point - (m_pos + Vector3D(halfSize, halfSize, halfSize));//Need to do this around origin
		double x = abs(newP.x());
		double y = abs(newP.y());
		double z = abs(newP.z());

		if(x > y && x > z) {
			normal = Vector3D(copysign(1.0,newP.x()),0.0,0.0);
		} else if(y > x && y > z) {
			normal = Vector3D(0.0,copysign(1.0,newP.y()),0.0);
		} else if(z > x && z > y) {
			normal = Vector3D(0.0, 0.0, copysign(1.0,newP.z()));
		} else {
			normal = Vector3D(0.0,0.0,0.0);
		}

		return true;
	}
	else {
		return false;
	}
}

Torus::~Torus()
{
}

bool Torus::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
	double A = 2.0, B = 1.0;
	Point3D E = ray.origin();
	Vector3D D = ray.direction();

	double G = 4 * A*A * (E.x()*E.x() + E.y()*E.y());
	double H = 8 * A*A * (D.x()*E.x() + D.y()*E.y());
	double I = 4 * A*A * (D.x()*D.x() + D.y()*D.y());
	double J = E.lengthSquared();
	double K = 2 * (dot(D, E));
	double L = D.lengthSquared() + (A*A + B*B);

	double u4 = J*J;
	double u3 = 2*J*K;
	double u2 = 2*J*L + K*K - G;
	double u1 = 2*K*L - H;
	double c = L*L - I;

  // // Solve quartic...
  double roots[4];
  int nroots = quarticRoots(u3/u4,u2/u4,u1/u4,c/u4,roots);

  // double intersections[4];
  // int num_intersections = 0;
  // while(nroots--)
  //   {
  //     float t = roots[nroots];
  //     float x = ray.origin().x() + t*ray.direction().x();
  //     float y = ray.origin().y() + t*ray.direction().y();
  //     float l = R*(M_PI/2 - atan2(y,x));
  //     if (l >= 0) //if (l <= vlength && l >= 0) what is vlength
  //       intersections[num_intersections++] = t;
  //   }	
 // cout << u4 << " " << u3 << " " << u2 << " " << u1 << " " << c << endl;

 	t = DBL_INF;
 	bool intersects = false;
 	for(int i = 0; i < nroots; i++) {
 		cout << roots[i] << endl;
 		if(roots[i] > MIN_INTERSECT_DIST && roots[i] < t) {
 			t = roots[i];
 			intersects = true;
 		}
 	}

 	return intersects;

}
// bool Torus::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point) {
//   // (the dot product) looks just right (:-)
//   struct { float a,b; } a;
//   a.b = dot(ray.origin(), ray.direction());
//   a.a = ray.origin().lengthSquared();

//   // Set up quartic in t:
//   //
//   //  4     3     2
//   // t + A t + B t + C t + D = 0
//   //
//   float R = 2.0;
//   float r = 1.0;
//   float R2 = R*R;
//   float K = a.a - r*r - R2;
//   float A = 4*a.b;
//   float B = 2*(2*a.b*a.b + K + 2*R2*ray.direction().z()*ray.direction().z());
//   float C = 4*(K*a.b + 2*R2*ray.origin().z()*ray.direction().z());
//   float D = K*K + 4*R2*(ray.origin().z()*ray.origin().z() - r*r);

//   // Solve quartic...
//   double roots[4];
//   int nroots = quarticRoots(A,B,C,D,roots);

//   double intersections[4];
//   int num_intersections = 0;
//   while(nroots--)
//     {
//       float t = roots[nroots];
//       float x = ray.origin().x() + t*ray.direction().x();
//       float y = ray.origin().y() + t*ray.direction().y();
//       float l = R*(M_PI/2 - atan2(y,x));
//       if (l >= 0) //if (l <= vlength && l >= 0) what is vlength
//         intersections[num_intersections++] = t;
//     }

//  	t = DBL_INF;
//  	bool intersects = false;
//  	for(int i = 0; i < num_intersections; i++) {
//  		if(intersections[i] > MIN_INTERSECT_DIST && intersections[i] < t) {
//  			t = intersections[i];
//  			intersects = true;
//  		}
//  	}

//  	return intersects;

// }


