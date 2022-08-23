#include "Transform.h"

Matrix4f MyTransform::uniformScale(float s)
{
	Matrix4f scaleMatrix = Matrix4f::Identity();
	scaleMatrix(0, 0) = s;
	scaleMatrix(1, 1) = s;
	scaleMatrix(2, 2) = s;
	return scaleMatrix;
}

Matrix4f MyTransform::rotate(Vector3f axis, float radian)
{
	Matrix4f rotationMatrix = Matrix4f::Identity();
	axis.normalize();
	Eigen::AngleAxisf rotationVector(radian, axis);
	rotationMatrix.topLeftCorner(3, 3) = rotationVector.matrix();
	return rotationMatrix;
}

Matrix4f MyTransform::translate(Vector3f t)
{
	Matrix4f translateMatrix = Matrix4f::Identity();
	translateMatrix.topRightCorner(3, 1) = t;
	return translateMatrix;
}

void MyTransform::print(const Matrix4f& m)
{
	qDebug() << m(0, 0) << " " << m(0, 1) << "" << m(0, 2) << " " << m(0, 3);
	qDebug() << m(1, 0) << " " << m(1, 1) << "" << m(1, 2) << " " << m(1, 3);
	qDebug() << m(2, 0) << " " << m(2, 1) << "" << m(2, 2) << " " << m(2, 3);
	qDebug() << m(3, 0) << " " << m(3, 1) << "" << m(3, 2) << " " << m(3, 3);
}

void MyTransform::print(const Matrix3f& m)
{
	qDebug() << m(0, 0) << " " << m(0, 1) << "" << m(0, 2);
	qDebug() << m(1, 0) << " " << m(1, 1) << "" << m(1, 2);
	qDebug() << m(2, 0) << " " << m(2, 1) << "" << m(2, 2);
}

Vector4f MyTransform::transformPoint(const Matrix4f& m, const Vector4f& p)
{
	return m * p;
}

Vector4f MyTransform::transformVector(const Matrix4f& m, const Vector4f& p)
{
	Vector4f temp(p.x(), p.y(), p.z(), 0);
	return m * temp;
}

Vector3f MyTransform::transformNormal(const Matrix4f& m, const Vector3f& p)
{
	// doesn't support affine transformation, thus doesn't need inverse transpose matrix
	Vector4f temp(p.x(), p.y(), p.z(), 0);
	return (m * temp).head(3).normalized();
}
