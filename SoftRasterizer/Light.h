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
	// 0 for directional light, 1 for point light
	// directional light for default
	DirectionalLight(int lightType=0, const Vector3f& p = Vector3f::Zero(), const Vector3f& d = Vector3f::Zero(), const Vector3f c = Vector3f::Zero());
	~DirectionalLight();

	void setDirectionalLightShadowArea(float near, float far, float left, float right, float bottom, float top);
	void setPointLightLightShadowArea(float near, float far, float vfov, float ratio);
	void updateCameraPose();

	void toPointLight();
	void toDirectionalLight();

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
	int lightType;
	//int depthBufferWidth;
};

#endif // !LIGHT_H
