#ifndef INPUT_H
#define INPUT_H

#include "Eigen/Geometry"
#include "Light.h"
using namespace Eigen;

struct ConstantBuffer
{
	DirectionalLight mainLight;

	Vector3f cameraPosition;
	Matrix4f objectToWorld;
	Matrix4f worldToCamera;
	Matrix4f projection;
	Matrix4f shadowView;
	Matrix4f shadowProj;
	float* shadowMap;
};

struct VertexInput {
	Vector4f positionOS;
	Vector3f normalOS;
	Vector2f texCoord;
};

struct FragmentInput {
	Vector4f positionWS;
	Vector4f positionCS;
	Vector3f normalWS;
	Vector2f texCoord;
	Vector4f shadowCoord;
};

#endif // !INPUT_H
