#include "primitive.hpp"
#include "polyroots.hpp"
#include "ray.hpp"
#include <vector>

Primitive::~Primitive()
{
}

bool Primitive::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
	return false;
}

Sphere::~Sphere()
{
}

bool Sphere::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
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

		double u = 0.5 + atan2(point.z(), -point.x()) / (2.0 * M_PI);
		double v = 0.5 - asin(point.y()) / M_PI;
		uv = Point2D(u,v);

		// if(c < 0.0 ) {
		// 	//cout << "w" << endl;
		// 	normal = -normal;
		// }

		return true;
	} 
}

void Sphere::getTangents(const Intersection &i, Vector3D &Ou, Vector3D &Ov) const {
	Ou = cross(i.normal, Vector3D(0,1,0));
	Ov = cross(Vector3D(1,0,0), i.normal); //* abs(sin(M_PI * (0.5 - v)));
}

Cube::~Cube()
{
}

bool Cube::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
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
	    int dir = 0;
	    for (int i = 0; i < 3; ++i) {
	        float distance = abs(0.5 - abs(newP[i]));

	        if (distance < min_distance) {
	            min_distance = distance;

	            normal = Vector3D();
	            normal[i] = copysign(1.0, newP[i]);
	            dir = i;
	        }
	    }
		
		double u,v;
		if(dir == 0) {
			u = (newP.z() + 0.5);
 			v = (newP.y() + 0.5);
		}
		else if(dir == 1) {
 			u = (newP.x() + 0.5);
 			v = (newP.z() + 0.5);
		}
		else {
 			u = (newP.x() + 0.5);
 			v = (newP.y() + 0.5);
 		}

 		uv = Point2D(u,v);

		//if(dot(normal, ray.direction()) > 0.0) normal = -normal; //TODO

		return true;
	}
	else {
		return false;
	}
}

Plane::~Plane()
{
}

bool Plane::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
	//Find point on plane
	normal = Vector3D(0,1,0);
	double lDn = dot(ray.direction(), normal);
	t = dot(-ray.origin(), normal) / lDn;
	point = ray.getPoint(t);

	if(t > MIN_INTERSECT_DIST) {
		if(point.lengthSquared() < mRadSq){
			double intp;
			double x = modf(point.x(), &intp);
			double z = modf(point.z(), &intp);

			if(x < 0) {
				x = 1.0 + x;
			}

			if(z < 0) {
				z = 1.0 + z;
			}			

			uv = Point2D(x,z);

			return true;
		}
	}

	return false;
}

void Plane::getTangents(const Intersection &i, Vector3D &Ou, Vector3D &Ov) const {
	Ou = Vector3D(-1.0,0.0,0.0); //???? ---???
	Ov = Vector3D(0.0,0.0,-1.0);
}

NonhierSphere::~NonhierSphere()
{
}

//TODO idea -- maybe I hsould only return t?
bool NonhierSphere::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
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

		Point3D object_point = point - m_pos;
		normal = object_point;
		normal.normalize();

		double u = 0.5 + atan2(object_point.z(), -object_point.x()) / (2.0 * M_PI);
		double v = 0.5 - asin(object_point.y()) / M_PI;
		uv = Point2D(u,v);

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

bool NonhierBox::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
	//Alg from http://tavianator.com/2011/05/fast-branchless-raybounding-box-intersections/
	double halfSize = m_size/2.0;
	Point3D minP = m_pos - Vector3D(halfSize, halfSize, halfSize);
	Point3D maxP = m_pos + Vector3D(halfSize, halfSize, halfSize);
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
		Point3D newP = point - m_pos;//Need to do this around origin
		double x = fabs(newP.x());
		double y = fabs(newP.y());
		double z = fabs(newP.z());

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

bool Torus::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
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



MengerSponge::MengerSponge(const Point3D &pos, int level, int maxLevel, double size) : mPos(pos), mLevel(level), mMaxLevel(maxLevel), mSize(size) {
	//mSize = pow(1.0/3.0,  level);//Todo profile with this change, don't have to store msize either, only need to compute once per level
    mBox = NonhierBox(pos, mSize);
};

MengerSponge::~MengerSponge()
{
}

bool MengerSponge::rayIntersection(const Ray &ray, double &t, Vector3D &normal, Point3D &point, Point2D &uv) {
	bool hitBox = mBox.rayIntersection(ray, t, normal, point, uv);

	if(hitBox){
		if(mLevel == mMaxLevel) {
			return hitBox;
		}

		vector<MengerSponge> sponges;
		sponges.reserve(20);

		double offset = mSize / 3.0;
		double newSize = pow(0.33333333333333333333333, mLevel + 1);
		sponges.push_back(MengerSponge(mPos + Point3D(-offset, offset, offset), mLevel + 1, mMaxLevel, newSize)); //1	
		sponges.push_back(MengerSponge(mPos + Point3D(-offset, offset, 0), mLevel + 1, mMaxLevel, newSize));	//2
		sponges.push_back(MengerSponge(mPos + Point3D(-offset, offset, -offset), mLevel + 1, mMaxLevel, newSize));//3	
		sponges.push_back(MengerSponge(mPos + Point3D(0, offset, -offset), mLevel + 1, mMaxLevel, newSize));	//4
		sponges.push_back(MengerSponge(mPos + Point3D(offset, offset, -offset), mLevel + 1, mMaxLevel, newSize));//5	
		sponges.push_back(MengerSponge(mPos + Point3D(offset, offset, 0), mLevel + 1, mMaxLevel, newSize));	//6
		sponges.push_back(MengerSponge(mPos + Point3D(offset, offset, offset), mLevel + 1, mMaxLevel, newSize));//7	
		sponges.push_back(MengerSponge(mPos + Point3D(0, offset, offset), mLevel + 1, mMaxLevel, newSize));	//8

		sponges.push_back(MengerSponge(mPos + Point3D(-offset, 0, offset), mLevel + 1, mMaxLevel, newSize)); //1	
		sponges.push_back(MengerSponge(mPos + Point3D(-offset, 0, -offset), mLevel + 1, mMaxLevel, newSize));//3	
		sponges.push_back(MengerSponge(mPos + Point3D(offset, 0, -offset), mLevel + 1, mMaxLevel, newSize));//5	
		sponges.push_back(MengerSponge(mPos + Point3D(offset, 0, offset), mLevel + 1, mMaxLevel, newSize));//7

		sponges.push_back(MengerSponge(mPos + Point3D(-offset, -offset, offset), mLevel + 1, mMaxLevel, newSize)); //1	
		sponges.push_back(MengerSponge(mPos + Point3D(-offset, -offset, 0), mLevel + 1, mMaxLevel, newSize));	//2
		sponges.push_back(MengerSponge(mPos + Point3D(-offset, -offset, -offset), mLevel + 1, mMaxLevel, newSize));//3	
		sponges.push_back(MengerSponge(mPos + Point3D(0, -offset, -offset), mLevel + 1, mMaxLevel, newSize));	//4
		sponges.push_back(MengerSponge(mPos + Point3D(offset, -offset, -offset), mLevel + 1, mMaxLevel, newSize));//5	
		sponges.push_back(MengerSponge(mPos + Point3D(offset, -offset, 0), mLevel + 1, mMaxLevel, newSize));	//6
		sponges.push_back(MengerSponge(mPos + Point3D(offset, -offset, offset), mLevel + 1, mMaxLevel, newSize));//7	
		sponges.push_back(MengerSponge(mPos + Point3D(0, -offset, offset), mLevel + 1, mMaxLevel, newSize));	//8

		
		hitBox = false;
		double ct = DBL_INF;
		Vector3D cn;
		Point3D cp;

		for (int i = 0; i < 20; i++) {
			bool intersects = sponges[i].rayIntersection(ray, t, normal, point, uv); //TODO should I perturb here? //ray.perturbed(MY_EPSILON)

			if(intersects) {
				hitBox = true;
				if(t < ct && t > MIN_INTERSECT_DIST) {
					ct = t;
					cn = normal;
					cp = point;
				}
			}
		}

		t = ct;
		normal = cn;
		point = cp;

		return hitBox;
	}

	return false;
}

// Intersection Renderer::findClosestIntersection(const Ray &ray, bool includeLights) const {
// 	Intersection closestI;
// 	closestI.ray = &ray;

// 	for (auto nodeIt = mGeometryList.begin(); nodeIt != mGeometryList.end(); ++nodeIt) {
// 		if(includeLights || !(*nodeIt).isLight()) {
// 			Intersection newI;
// 			bool intersects = nodeIt->computeIntersection(ray, newI); //TODO should I perturb here? //ray.perturbed(MY_EPSILON)

// 			if(intersects) {
// 				if(newI.t < closestI.t && newI.t > MIN_INTERSECT_DIST) {
// 					closestI = newI;
// 				}
// 			}
// 		}
// 	}

// 	return closestI;
// }



