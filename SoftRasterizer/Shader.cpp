#include "Shader.h"

Vector2f poissonDisk[100];
bool enableSoftShadow = false;
int numSamples = 5;
int numRings = 5;
int filterSize = 5;
int shadowMapWidth = 1024;

void setSoftShadow(bool enable) {
	enableSoftShadow = enable;
}

void setNumSamples(int n) {
	numSamples = n;
}

void setNumRings(int n) {
	numRings = n;
}

void setFilterSizse(int n) {
	filterSize = n;
}

void setShadowMapWidth(int w) {
	shadowMapWidth = w;
}

template<typename T>
T lerp(const T& a, const T& b, float s) {
	return (1.0 - s) * a + s * b;
}

float saturate(float a) {
	return std::min(std::max(0.0f, a), 1.0f);
}

float mod(float a, float b) {
	return a - b * std::floor(a / b);
}

float fract(float a) {
	return a - std::floor(a);
}

float rand_1to1(float x) {
	// -1 -1
	return fract(sin(x) * 10000.0);
}

float rand_2to1(const Vector2f& uv) {
	// 0 - 1
	const float a = 12.9898, b = 78.233, c = 43758.5453;
	//float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	float dt = uv.dot(Vector2f(a, b));
	float sn = mod(dt, M_PI);
	return fract(std::sin(sn) * c);
}

void uniformDiskSamples(const Vector2f& randomSeed) {

	float randNum = rand_2to1(randomSeed);
	float sampleX = rand_1to1(randNum);
	float sampleY = rand_1to1(sampleX);

	float angle = sampleX * M_2_PI;
	float radius = sqrt(sampleY);

	//for (int i = 0; i < NUM_SAMPLES; i++) {
	for (int i = 0; i < numSamples; i++) {
		poissonDisk[i] = Vector2f(radius * std::cos(angle), radius * std::sin(angle));

		sampleX = rand_1to1(sampleY);
		sampleY = rand_1to1(sampleX);

		angle = sampleX * M_2_PI;
		radius = std::sqrt(sampleY);
	}
}

void poissonDiskSamples(const Vector2f& randomSeed)
{
	//float ANGLE_STEP = M_2_PI * float(NUM_RINGS) / float(NUM_SAMPLES);
	//float INV_NUM_SAMPLES = 1.0 / float(NUM_SAMPLES);
	float ANGLE_STEP = M_2_PI * float(numRings) / float(numSamples);
	float INV_NUM_SAMPLES = 1.0 / float(numSamples);

	float angle = rand_2to1(randomSeed) * M_2_PI;

	float radius = INV_NUM_SAMPLES;
	float radiusStep = radius;

	//for (int i = 0; i < NUM_SAMPLES; i++) {
	for (int i = 0; i < numSamples; i++) {
		poissonDisk[i] = Vector2f(cos(angle), sin(angle)) * pow(radius, 0.75);
		radius += radiusStep;
		angle += ANGLE_STEP;
	}
}

float findBlocker(float* shadowMap, const Vector2f& uv, float zReceiver) {

	//const int shadowMapWidth = 1024;
	int count = 0;
	float blockerDepth = 0.0;
	//for (int i = 0; i < NUM_SAMPLES; ++i) {
	for (int i = 0; i < numSamples; ++i) {
		//vec4 rgbaDepth = texture2D(shadowMap, uv + poissonDisk[i] * 0.005);
		//float depth = unpack(rgbaDepth);
		int index = uv.x() + (shadowMapWidth - 1 - uv.y()) * shadowMapWidth;
		float depth = shadowMap[index];
		//if (depth < EPS) depth = 1.0;
		if (zReceiver > depth + 0.005) {
			blockerDepth += depth;
			count++;
		}
	}
	return blockerDepth / float(count);
}

float PCF(const float* shadowMap, int shadowWidth, const Vector2f& uv, float depth)
{
	//const int shadowMapWidth = 1024;
	//Vector2f seed(rand() / double(RAND_MAX), rand() / double(RAND_MAX));
	//poissonDiskSamples(poissonDisk, seed, 10, numSamples);
	//for (int i = 0; i < numSamples; ++i) {
	//	qDebug("poissonDisk[%d] = (%f, %f)", i, poissonDisk[i].x(), poissonDisk[i].y());
	//}
	int count = 0;
	for (int i = 0; i < numSamples; ++i) {

		//Vector2f sampleUV = uv + poissonDisk[i]*filterSize;
		//Vector2f sampleUV = uv;
		int x = uv.x() * shadowMapWidth;
		int y = uv.y() * shadowMapWidth;
		x += poissonDisk[i].x() * filterSize;
		y += poissonDisk[i].y() * filterSize;
		x = std::max(0, x);
		x = std::min(shadowMapWidth - 1, x);
		y = std::max(0, y);
		y = std::min(shadowMapWidth - 1, y);

		int index = x + (shadowMapWidth - 1 - y) * shadowMapWidth;
		float sampledDepth = shadowMap[index];
		if (depth >= sampledDepth) ++count;
		//for (int i = 0; i < numSamples; ++i) {
		//	qDebug("(x, y) = (%f, %f)", x, y);
		//	qDebug("depth = %f, sampledDepth = %f", depth, sampledDepth);
		//}
	}
	//qDebug("count = %d, numSamples = %d", count, numSamples);
	return float(count) / float(numSamples);
}

float sampleShadowMap(const float* shadowMap, int shadowWidth, const Vector2f& uv, float depth, float NdotL)
{
	//const int shadowMapWidth = 1024;

	int x = uv.x() * shadowMapWidth;
	int y = uv.y() * shadowMapWidth;

	if (x < 0 || x >= shadowMapWidth || y < 0 || y >= shadowMapWidth) {
		return 1;
	}

	y = shadowMapWidth - 1 - y;


	float bias = std::max(0.001f, 1.0f / 512 * (1.0f - NdotL));


	return PCF(shadowMap, shadowWidth, uv, depth + bias);
	float shadowDepth = shadowMap[x + y * shadowMapWidth];
	return depth + bias >= shadowDepth;

	if (enableSoftShadow) {
		return PCF(shadowMap, shadowWidth, uv, depth + bias);
	}
	else {
		float shadowDepth = shadowMap[x + y * shadowMapWidth];
		return depth + bias >= shadowDepth;
	}
}

FragmentInput CommonVertexShader(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput& input)
{
	FragmentInput output;
	output.positionWS = cb.objectToWorld * input.positionOS;
	// doesn't support affine transformation, so doesn't need inverse transpose matrix
	output.normalWS = cb.objectToWorld.topLeftCorner(3, 3) * input.normalOS;

	//if (output.normalWS.x() == 0 && output.normalWS.y() == 0 && output.normalWS.z() == 0) {
		//qDebug("input.normalOS = (%f, %f, %f)", input.normalOS.x(), input.normalOS.y(), input.normalOS.z());
		//MyTransform::print(cb.objectToWorld.topLeftCorner(3, 3));
	//}
	output.positionCS = cb.projection * cb.worldToCamera * output.positionWS;

	Vector4f shadowCS = cb.lightShadowProjMatrix * cb.lightShadowViewMatrix * output.positionWS;
	output.shadowCoord.head(2) = (shadowCS.head(2) / shadowCS.w() * 0.5f) + Vector2f(0.5f, 0.5);
	output.shadowCoord.tail(2) = shadowCS.tail(2);
	return output;
}

Vector3f BlinnPhongFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input)
{
	//return Vector3f();
	Vector3f lightDir = cb.lightDir.normalized();
	Vector3f viewDir = (cb.cameraPosition - input.positionWS.head(3)).normalized();

	Vector3f H = (lightDir + viewDir).normalized();
	float NdotH = input.normalWS.dot(H);
	float NdotL = input.normalWS.dot(lightDir);
	NdotH = std::max(0.0f, NdotH);
	NdotL = std::max(0.0f, NdotL);

	float shadowAttenuation = sampleShadowMap(cb.lightShadowMap, cb.lightShadowWidth, input.shadowCoord.head(2), input.shadowCoord.z(), NdotL);
	float distanceAttenuation = 1.0f;
	//Vector3f attenuatedLightColor = mainLight.color * (mainLight.distanceAttenuation * mainLight.shadowAttenuation) * NdotL;
	Vector3f attenuatedLightColor = cb.lightCol * (distanceAttenuation * shadowAttenuation) * NdotL;

	float smooth = std::pow(2.0f, sp.smoothness + 1);
	Vector3f specularColor = std::pow(NdotH, smooth) * (attenuatedLightColor.array() * sp.specular.array()).matrix();
	//qDebug("specularColor = (%f, %f, %f)", specularColor.x(), specularColor.y(), specularColor.z());

	//Vector3f diffuseColor = (attenuatedLightColor.array() * cb.albedo.array()).matrix();
	Vector3f diffuseColor = (attenuatedLightColor.array() * sp.albedo.array()).matrix();
	Vector3f ambientColor = (sp.ambientColor.array() * sp.albedo.array()).matrix();

	//return diffuseColor;
	return specularColor + diffuseColor + ambientColor;
	//Vector3f color = specularColor + diffuseColor;
	//color.x() = std::max(1.0f, color.x());
	//color.y() = std::max(1.0f, color.y());
	//color.z() = std::max(1.0f, color.z());
	//return color;
}


float PerceptualSmoothnessToPerceptualRoughness(float perceptualSmoothness)
{
	return (1.0 - perceptualSmoothness);
}

float PerceptualRoughnessToRoughness(float perceptualRoughness)
{
	return perceptualRoughness * perceptualRoughness;
}

float OneMinusReflectivityMetallic(float metallic)
{
	// We'll need oneMinusReflectivity, so
	//   1-reflectivity = 1-lerp(dielectricSpec, 1, metallic) = lerp(1-dielectricSpec, 0, metallic)
	// store (1-dielectricSpec) in kDielectricSpec.a, then
	//   1-reflectivity = lerp(alpha, 0, metallic) = alpha + metallic*(0 - alpha) =
	//                  = alpha - metallic * alpha
	float oneMinusDielectricSpec = (1.0f - 0.08);
	return oneMinusDielectricSpec - metallic * (1.0f - oneMinusDielectricSpec);
}


void InitializeBRDFData(BRDFData& brdfData, const ShaderProperties& sp) {
	brdfData.albedo = sp.albedo;

	//float reflectivity = sp.specColor.maxCoeff();
	//float oneMinusReflectivity = 1.0f - reflectivity;

	float oneMinusReflectivity = OneMinusReflectivityMetallic(sp.metallic);
	float reflectivity = half(1.0) - oneMinusReflectivity;

	brdfData.diffuse = sp.albedo * oneMinusReflectivity;
	Vector3f kDieletricSpec(0.08, 0.08, 0.08);
	brdfData.specular = lerp(kDieletricSpec, sp.albedo, sp.metallic);

	brdfData.perceptualRoughness = PerceptualSmoothnessToPerceptualRoughness(sp.smoothness);
	brdfData.roughness = std::max(PerceptualRoughnessToRoughness(brdfData.perceptualRoughness), float(HALF_MIN_SQRT));
	brdfData.roughness2 = std::max(brdfData.roughness * brdfData.roughness, float(HALF_MIN));
	brdfData.grazingTerm = saturate(sp.smoothness + brdfData.reflectivity);
	brdfData.normalizationTerm = brdfData.roughness2 * 4.0f + 2.0f;
	brdfData.roughness2MinusOne = brdfData.roughness2 - 1.0f;
}

// Computes the scalar specular term for Minimalist CookTorrance BRDF
// NOTE: needs to be multiplied with reflectance f0, i.e. specular color to complete
float DirectBRDFSpecular(const BRDFData& brdfData, const Vector3f& normalWS, const Vector3f& lightDirectionWS, const Vector3f& viewDirectionWS)
{
	Vector3f halfDir = (lightDirectionWS + viewDirectionWS).normalized();

	float NoH = saturate(normalWS.dot(halfDir));
	float LoH = float(saturate(lightDirectionWS.dot(halfDir)));

	// GGX Distribution multiplied by combined approximation of Visibility and Fresnel
	// BRDFspec = (D * V * F) / 4.0
	// D = roughness^2 / ( NoH^2 * (roughness^2 - 1) + 1 )^2
	// V * F = 1.0 / ( LoH^2 * (roughness + 0.5) )
	// See "Optimizing PBR for Mobile" from Siggraph 2015 moving mobile graphics course
	// https://community.arm.com/events/1155

	// Final BRDFspec = roughness^2 / ( NoH^2 * (roughness^2 - 1) + 1 )^2 * (LoH^2 * (roughness + 0.5) * 4.0)
	// We further optimize a few light invariant terms
	// brdfData.normalizationTerm = (roughness + 0.5) * 4.0 rewritten as roughness * 4.0 + 2.0 to a fit a MAD.
	float d = NoH * NoH * brdfData.roughness2MinusOne + 1.00001f;
	float d2 = float(d * d);

	float LoH2 = LoH * LoH;
	float specularTerm = brdfData.roughness2 / (d2 * std::max(0.1f, LoH2) * brdfData.normalizationTerm);

	// On platforms where half actually means something, the denominator has a risk of overflow
	// clamp below was added specifically to "fix" that, but dx compiler (we convert bytecode to metal/gles)
	// sees that specularTerm have only non-negative terms, so it skips max(0,..) in clamp (leaving only min(100,...))

	return specularTerm;
}

Vector3f UnityPBRFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input)
{
	BRDFData brdfData;
	InitializeBRDFData(brdfData, sp);

	float NdotL = saturate(input.normalWS.dot(cb.lightDir));
	//float NdotL = input.normalWS.dot(cb.mainLight.direction);

	float shadowAttenuation = sampleShadowMap(cb.lightShadowMap, cb.lightShadowWidth, input.shadowCoord.head(2), input.shadowCoord.z(), NdotL);
	//float shadowAttenuation = 1.0f;


	//Vector3f radiance = cb.mainLight.color * cb.mainLight.distanceAttenuation * cb.mainLight.shadowAttenuation * NdotL;
	float distanceAttenuation = 1.0f;
	Vector3f radiance = cb.lightCol * distanceAttenuation * shadowAttenuation * NdotL;
	Vector3f brdf = brdfData.diffuse;

	Vector3f viewDir = (cb.cameraPosition - input.positionWS.head(3)).normalized();
	brdf += brdfData.specular * DirectBRDFSpecular(brdfData, input.normalWS, cb.lightDir, viewDir);


	//float temp = DirectBRDFSpecular(brdfData, input.normalWS, cb.mainLight.direction, viewDir);
	//return brdfData.specular * temp;
	//return brdf;
	//return brdfData.specular;
	return (brdf.array() * radiance.array()).matrix();
}

FragmentInput ShadowMapVertexShader(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput& input)
{
	FragmentInput output;
	output.positionWS = cb.objectToWorld * input.positionOS;
	output.positionCS = cb.projection * cb.worldToCamera * output.positionWS;
	output.normalWS = cb.objectToWorld.topLeftCorner(3, 3) * input.normalOS;
	return output;
}

Vector3f ShadowMapFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input)
{
	return Vector3f(input.positionCS.z(), 0, 0);
}
