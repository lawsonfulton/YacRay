#include "camera.hpp"

Camera::Camera(Point3D eye, Vector3D view, Vector3D up, double fov, int width, int height)
        : mLookFrom(eye), mViewDirection(view), mUp(up), mFov(fov), mWidth(width), mHeight(height)
{
    double d = 1.0;
    double worldH = 2.0 * d * tan(mFov/2.0 * M_PI/180.0); //Remeber have to convert to rads!!!
    double worldW = (double)mWidth / (double)mHeight * worldH;

    //1. Translate
    Matrix4x4 T1;
    T1.translate(Vector3D(-mWidth/2.0, -mHeight/2, d));

    //2. Scale
    Matrix4x4 S2;
    S2.scale(Vector3D(-worldW/mWidth, -worldH/mHeight, 1.0)); //Need to flip x axis

    //3. Rotate
    Matrix4x4 R3;
    Vector3D w = (mViewDirection).normalized();
    Vector3D u = cross(mUp, w).normalized();
    Vector3D v = cross(w, u);

    R3.setColumn(0, Vector4D(u, 0.0));
    R3.setColumn(1, Vector4D(v, 0.0));
    R3.setColumn(2, Vector4D(w, 0.0));
    R3.setColumn(3, Vector4D(0.0, 0.0, 0.0, 1.0));

    //4. Translate
    Matrix4x4 T4;
    T4.translate(mLookFrom);

    mScreenToWorld = T4 * R3 * S2 * T1;
}

Ray Camera::makeRay(Point2D screenCord) {
    Point3D pixelCoord(screenCord, 0.0);
    Point3D worldCoord = mScreenToWorld * pixelCoord;

    return Ray(mLookFrom, (worldCoord - mLookFrom).normalized());
}

// #include "camera.hpp"

// Camera::Camera(Point3D eye, Vector3D view, Vector3D up, double fov, int width, int height)
//         : mLookFrom(eye), mLookAt(view), mUp(up), mFov(fov), mWidth(width), mHeight(height)
// {
//     double d = 1.0;
//     double worldH = 2.0 * d * tan(mFov/2.0);
//     double worldW = (double)mWidth / (double)mHeight * worldH;

//     //1. Translate
//     Matrix4x4 T1;
//     T1.translate(Vector3D(-mWidth/2.0, -mHeight/2, d)); //Can z be anything (d)?

//     //2. Scale
//     Matrix4x4 S2;
//     S2.scale(Vector3D(-worldH/mHeight, worldW/mWidth, 1.0));

//     //3. Rotate
//     Matrix4x4 R3;
//     Vector3D w = (mLookAt - mLookFrom).normalized();
//     Vector3D u = cross(mUp, w).normalized();
//     Vector3D v = cross(w, u);

//     R3.setColumn(0, Vector4D(u, 0.0));
//     R3.setColumn(1, Vector4D(v, 0.0));
//     R3.setColumn(2, Vector4D(w, 0.0));
//     R3.setColumn(3, Vector4D(0.0, 0.0, 0.0, 1.0));

//     //4. Translate
//     Matrix4x4 T4;
//     T4.translate(mLookFrom);

//     mScreenToWorld = T4 * R3 * S2 * T1;
// }

// Ray Camera::makeRay(int x, int y) {
//     Point3D pixelCoord((double)x, (double)y, 0.0);
//     Point3D worldCoord = mScreenToWorld * pixelCoord;

//     return Ray(mLookFrom, worldCoord - mLookFrom);
// }