#ifndef LIGHT_H
#define LIGHT_H

#include "Eigen/Core"
#include <memory>
#include "Camera.h"
#include "Transform.h"
#include "MathDefines.h"
#include "FastRenderer.h"

//#include "tiny_obj_loader.h"
using namespace Eigen;

class Camera;

struct DirectionalLight
{
	DirectionalLight(const Vector3f& p = Vector3f::Zero(), const Vector3f& d = Vector3f::Zero(), const Vector3f c = Vector3f::Zero());
	~DirectionalLight();

	void setShadowArea(float near, float far, float left, float right, float bottom, float top);
	void updateCameraPose();

	void createRenderer();
	void disposeRenderer();

	void bindReader(tinyobj::ObjReader* reader);
	void disposeReader();

	void bindDepthBuffer(float* db, int w);
	void disposeDepthBuffer();

	//void bindConstantBuffer(ConstantBuffer* cb);
	void bindConstantBuffer(const ConstantBuffer& cb);
	void disposeConstantBuffer();

	Camera* camera;
	//float* depthBuffer;

	Vector3f position;
	Vector3f direction;
	Vector3f color;
	//int depthBufferWidth;
};

//struct PointLight
//{
//	Vector3f position;
//	Vector3f color;
//	Camera* cam = nullptr;
//};

#endif // !LIGHT_H
