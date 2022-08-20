#include "Camera.h"

void Camera::setPosition(const Vector3f& p)
{
	pos = p;
}

void Camera::setTarget(const Vector3f& t)
{
	target = t;
}

void Camera::setUp(const Vector3f& u)
{
	up = u;
}

void Camera::setVFov(float f)
{
	vfov = f;
}

void Camera::setRatio(float r)
{
	ratio = r;
}

void Camera::setNear(float n)
{
	near = n;
}

void Camera::setFar(float f)
{
	far = f;
}

void Camera::updateTransform()
{
	z = (target - pos).normalized();
	x = (up.cross(z)).normalized();
	y = z.cross(x);

	worldToCamera(0, 0) = x.x();
	worldToCamera(0, 1) = x.y();
	worldToCamera(0, 2) = x.z();

	worldToCamera(1, 0) = y.x();
	worldToCamera(1, 1) = y.y();
	worldToCamera(1, 2) = y.z();

	worldToCamera(2, 0) = z.x();
	worldToCamera(2, 1) = z.y();
	worldToCamera(2, 2) = z.z();

	worldToCamera(0, 3) = -x.dot(pos);
	worldToCamera(1, 3) = -y.dot(pos);
	worldToCamera(2, 3) = -z.dot(pos);


	cameraToWorld.topLeftCorner(3, 3) = worldToCamera.topLeftCorner(3, 3).transpose();
	cameraToWorld.topRightCorner(3, 1) = pos;

	//Matrix4f temp = worldToCamera * cameraToWorld;
	//MyTransform t;
	//t.m = temp;
	//t.print();
}

void Camera::updateProjection()
{
	float halfRadianFov = vfov * M_PI * 0.5 / 180.0;
	float c = 1.0 / std::tan(halfRadianFov);

	projection(0, 0) = c / ratio;
	projection(1, 1) = c;
	projection(2, 2) = -near / (far - near);
	projection(2, 3) = near * far / (far - near);
	projection(3, 2) = 1;
}

Matrix4f Camera::getWorldToCamera() const
{
	return worldToCamera;
}

Matrix4f Camera::getCameraToWorld() const
{
	return cameraToWorld;
}

Matrix4f Camera::getProjection() const
{
	return projection;
}

void Camera::rotate(Vector3f axis, float radian)
{
	Vector3f viewDir = target - pos;
	axis.normalize();
	Eigen::AngleAxisf rotationVector(radian, axis);
	Matrix3f rotationMatrix = rotationVector.matrix();
	Vector3f newViewDir = rotationMatrix * viewDir;
	up = rotationMatrix * up;
	target = pos + newViewDir;
}

void Camera::rotateAroundX(float radian)
{
	Vector3f axis(1, 0, 0);
	Vector3f viewDir = target - pos;
	axis.normalize();
	Eigen::AngleAxisf rotationVector(radian, axis);
	Matrix3f rotationMatrix = rotationVector.matrix();
	Vector3f newViewDir = rotationMatrix * viewDir;
	up = rotationMatrix * up;
	target = pos + newViewDir;
}

void Camera::rotateAroundY(float radian)
{
	Vector3f axis(0, 1, 0);
	Vector3f viewDir = target - pos;
	axis.normalize();
	Eigen::AngleAxisf rotationVector(radian, axis);
	Matrix3f rotationMatrix = rotationVector.matrix();
	Vector3f newViewDir = rotationMatrix * viewDir;
	up = rotationMatrix * up;
	target = pos + newViewDir;
}

void DirectionalLightShadowCamera::updateProjection()
{
	float halfRadianFov = vfov * M_PI * 0.5 / 180.0;
	float c = 1.0 / std::tan(halfRadianFov);

	c = c / near;

	projection(0, 0) = c / ratio;
	projection(1, 1) = c;
	//projection(2, 2) = 1 / (far - near);
	//projection(2, 3) = -near / (far - near);
	projection(2, 2) = -1 / (far - near);
	projection(2, 3) = far / (far - near);
	projection(3, 3) = 1;
}
