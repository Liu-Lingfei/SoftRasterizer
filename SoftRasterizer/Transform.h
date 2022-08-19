#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "qdebug.h"
using namespace Eigen;

class MyTransform
{
public:
	MyTransform() : m(Matrix4f::Identity()) {}
	Matrix4f m;

	void uniformScale(float s);
	void rotate(Vector3f axis, float radian);
	void translate(Vector3f t);
	void print();

	Vector4f transformPoint(const Vector4f& p);
	Vector4f transformVector(const Vector4f& p);
	Vector4f transformNormal(const Vector4f& p);
};

#endif // !TRANSFORM_H
