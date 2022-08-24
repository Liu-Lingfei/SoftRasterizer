#ifndef CONSTANT_BUFFER
#define CONSTANT_BUFFER

#include "Eigen/Core"
using namespace Eigen;

//#include "Camera.h"
//#include "Light.h"
//#include "Shader.h"

struct Material;
class DirectionalLight;
class Camera;


struct ConstantBuffer
{
	// global info
	Matrix4f objectToWorld;
	Material* material;

	// per camera info
	Vector3f cameraPosition;
	Matrix4f worldToCamera;
	Matrix4f projection;
	int cameraType;
	const float* depthBuffer;

	// per light info
	Vector3f lightPos;
	Vector3f lightDir;
	Vector3f lightCol;

	Matrix4f lightShadowViewMatrix;
	Matrix4f lightShadowProjMatrix;
	const float* lightShadowMap;
	int lightShadowWidth;
	int lightType;
	int lightNear;
	int lightFar;
};

#endif // !CONSTANT_BUFFER
