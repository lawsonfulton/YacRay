#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "algebra.hpp"
#include "ray.hpp"

class Camera {
public:
  	Camera(Point3D eye, Vector3D view, Vector3D up, double fov, int width, int height);

  	Ray makeRay(Point2D screenCoord);

  	const Point3D &getLocation() const { return mLookFrom; }

  	int width()  const { return mWidth;  }
  	int height() const { return mHeight; }

private:
	QMatrix4x4 mScreenToWorld;
	Point3D mLookFrom;
	Vector3D mViewDirection;
	Vector3D mUp;

	double mFov;
	int mWidth, mHeight;
};

#endif