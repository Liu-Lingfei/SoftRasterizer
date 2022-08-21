#ifndef SHADER_H
#define SHADER_H

#include "Eigen/Geometry"
#include "Input.h"
#include "MathDefines.h"
#include <QDebug>
#include <functional>
#include <cmath>
using namespace Eigen;

struct ShaderProperties {
	//Blinn-Phong
	Vector3f albedo;
	Vector3f specular;
	Vector3f ambientColor;

	//Unity
	//Vector3f albedo;
	float smoothness;
	float metallic;
};


typedef std::function<FragmentInput(const ConstantBuffer& cb, const ShaderProperties& sp, VertexInput&)> VertexShader;
typedef std::function<Vector3f(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput&)> FragmentShader;

float sampleShadowMap(const float* shadowMap, const Vector2f& uv, float depth);

FragmentInput CommonVertexShader(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput& input);
Vector3f BlinnPhongFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input);

struct BRDFData
{
	Vector3f albedo;
	Vector3f diffuse;
	Vector3f specular;
	float reflectivity;
	float perceptualRoughness;
	float roughness;
	float roughness2;
	float grazingTerm;

	float normalizationTerm;     // roughness * 4.0 + 2.0
	float roughness2MinusOne;    // roughness^2 - 1.0
};

Vector3f UnityPBRFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input);

FragmentInput ShadowMapVertexShader(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput& input);
Vector3f ShadowMapFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input);

#endif // !SHADER_H
