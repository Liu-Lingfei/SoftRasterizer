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
	//void setObjectToWorld(const Matrix4f& m);
	//void setMaterial(Material* m);
	//void setPerCameraInfo(const Camera& cam);
	//void setPerLightInfo(DirectionalLight& l);

	// global info
	Matrix4f objectToWorld;
	Material* material;

	// per camera info
	Vector3f cameraPosition;
	Matrix4f worldToCamera;
	Matrix4f projection;
	int cameraType;

	// per light info
	Vector3f lightPos;
	Vector3f lightDir;
	Vector3f lightCol;

	Matrix4f lightShadowViewMatrix;
	Matrix4f lightShadowProjMatrix;
	float* lightShadowMap;
	int lightShadowWidth;
};

#endif // !CONSTANT_BUFFER
