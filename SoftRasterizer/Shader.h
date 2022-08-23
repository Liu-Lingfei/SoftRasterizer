#ifndef SHADER_H
#define SHADER_H

#include "Eigen/Core"
#include "MathDefines.h"
using namespace Eigen;

void setSoftShadow(bool enable);
void setNumSamples(int n);
void setNumRings(int n);
void setFilterSizse(int n);
void setShadowMapWidth(int w);


float findBlocker(float* shadowMap, const Vector2f& uv, float zReceiver);
void uniformDiskSamples(const Vector2f& randomSeed);
void poissonDiskSamples(const Vector2f& randomSeed);
float PCF(const float* shadowMap, int shadowWidth, const Vector2f& uv, float depth);
float sampleShadowMap(const float* shadowMap, int shadowWidth, const Vector2f& uv, float depth, float NdotL);


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

struct ShaderProperties {
	// for Blinn-Phong
	Vector3f albedo = Vector3f(0.9, 0.5, 0.5);
	Vector3f specular = Vector3f(0.1, 0.1, 0.1);
	Vector3f ambientColor = Vector3f(0.1, 0.1, 0.1);

	// for Unity PBR
	//Vector3f albedo;
	float smoothness = 0.8;
	float metallic = 0.8;
};

#include "ConstantBuffer.h"

struct ConstantBuffer;

typedef FragmentInput(*VertexShader)(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput&);
typedef Vector3f(*FragmentShader)(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput&);

FragmentInput CommonVertexShader(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput& input);
Vector3f BlinnPhongFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input);

Vector3f UnityPBRFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input);

FragmentInput ShadowMapVertexShader(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput& input);
Vector3f ShadowMapFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input);

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

struct Material
{
	ShaderProperties shaderProps;
	VertexShader vertexShader = CommonVertexShader;
	FragmentShader fragmentShader = UnityPBRFragmentShader;
};


#endif // !SHADER_H
