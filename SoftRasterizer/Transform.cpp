#include "Transform.h"

void MyTransform::uniformScale(float s)
{
	m(0, 0) *= s;
	m(1, 1) *= s;
	m(2, 2) *= s;
}

void MyTransform::rotate(Vector3f axis, float radian)
{
	Matrix4f rotationMatrix = Matrix4f::Identity();
	axis.normalize();
	//float sine = std::sin(radian / 2);
	//float cosine = std::cos(radian / 2);
	//Quaternionf q;
	//q.x() = axis.x() * sine;
	//q.y() = axis.y() * sine;
	//q.z() = axis.z() * sine;
	//q.w() = cosine;
	//m.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();

	Eigen::AngleAxisf rotationVector(radian, axis);
	rotationMatrix.topLeftCorner(3, 3) = rotationVector.matrix();
	//m.topLeftCorner(3, 3) = rotationVector.matrix();
	m = rotationMatrix * m;
}

void MyTransform::translate(Vector3f t)
{
	m.topRightCorner(3, 1) += t;
}

void MyTransform::print()
{
	qDebug() << m(0, 0) << " " << m(0, 1) << "" << m(0, 2) << " " << m(0, 3);
	qDebug() << m(1, 0) << " " << m(1, 1) << "" << m(1, 2) << " " << m(1, 3);
	qDebug() << m(2, 0) << " " << m(2, 1) << "" << m(2, 2) << " " << m(2, 3);
	qDebug() << m(3, 0) << " " << m(3, 1) << "" << m(3, 2) << " " << m(3, 3);

}

Vector4f MyTransform::transformPoint(const Vector4f& p)
{
	return m * p;
}

Vector4f MyTransform::transformVector(const Vector4f& p)
{
	Vector4f temp(p.x(), p.y(), p.z(), 0);
	return m * temp;
}

Vector4f MyTransform::transformNormal(const Vector4f& p)
{
	// doesn't support affine transformation, thus doesn't need inverse transpose matrix
	Vector4f temp(p.x(), p.y(), p.z(), 0);
	return (m * temp).normalized();
}
