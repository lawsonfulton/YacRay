#include "ray.hpp"

Point3D Intersection::getPoint() const {
	return point;
}

void Ray::transform(const Matrix4x4 &mat) {
	Point3D p = mat * getPoint(1.0);

	mOrigin = mat * mOrigin;
	mDirection = (p - mOrigin).normalized();
}