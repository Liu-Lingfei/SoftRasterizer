#ifndef CAMERA_H
#define CAMERA_H

#include "Transform.h"
#include "Eigen/Geometry"
#include "MathDefines.h"
using namespace Eigen;

class Camera
{
public:
	Camera(const Vector3f& p=Vector3f::Zero(), const Vector3f& t = Vector3f::Zero(), const Vector3f& u =Vector3f::Zero())
		:
		pos(p), target(t), up(u),
		x(Vector3f::Zero()), y(Vector3f::Zero()), z(Vector3f::Zero()),
		vfov(0), ratio(0), near(0), far(0),
		worldToCamera(Matrix4f::Identity()),
		cameraToWorld(Matrix4f::Identity()),
		projection(Matrix4f::Zero())
	{}

	void setPosition(const Vector3f& p);
	void setTarget(const Vector3f& t);
	void setUp(const Vector3f& u);

	void setVFov(float f);
	void setRatio(float r);
	void setNear(float n);
	void setFar(float f);

	void updateTransform();
	void updateProjection();
	Matrix4f getWorldToCamera() const;
	Matrix4f getCameraToWorld() const;
	Matrix4f getProjection() const;

	void rotate(Vector3f axis, float radian);
	void rotateAroundX(float radian);
	void rotateAroundY(float radian);

	Vector3f pos;
	Vector3f target;
	Vector3f up;

	Vector3f x, y, z;

	// in degrees, not radians
	float vfov;
	// screenWidth/screenHeight
	float ratio;
	float near;
	float far;

	Matrix4f worldToCamera;
	Matrix4f cameraToWorld;
	Matrix4f projection;
};

class DirectionalLightShadowCamera : public Camera {
public:
	DirectionalLightShadowCamera(const Vector3f& p = Vector3f::Zero(), const Vector3f& t = Vector3f::Zero(), const Vector3f& u = Vector3f::Zero())
		:
		Camera(p, t, u),
		left(0),
		right(0),
		bottom(0),
		top(0)
	{}
	float left, right, bottom, top;
	void setLeft(float l) { left = l; }
	void setRight(float r) { right = r; }
	void setBottom(float b) { bottom = b; }
	void setTop(float t) { top = t; }
	void updateProjection();
};

#endif // !CAMERA_H
