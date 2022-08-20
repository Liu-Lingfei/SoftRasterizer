#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "qdebug.h"
using namespace Eigen;

class MyTransform
{
public:
	static Matrix4f uniformScale(float s);
	static Matrix4f rotate(Vector3f axis, float radian);
	static Matrix4f translate(Vector3f t);
	static void print(const Matrix4f& m);

	static Vector4f transformPoint(const Matrix4f& m, const Vector4f& p);
	static Vector4f transformVector(const Matrix4f& m, const Vector4f& p);
	static Vector3f transformNormal(const Matrix4f& m, const Vector3f& p);
};

#endif // !TRANSFORM_H
