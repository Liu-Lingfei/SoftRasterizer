#include "Shader.h"


FragmentInput CommonVertexShader(const ConstantBuffer& cb, const ShaderProperties& sp, const VertexInput& input)
{
	FragmentInput output;
	output.positionWS = cb.objectToWorld * input.positionOS;

	//qDebug("input.positionSS = %f, %f, %f)", input.positionOS.x(), input.positionOS.y(), input.positionOS.z());
	//qDebug("input.positionWS = %f, %f, %f)", output.positionWS.x(), output.positionWS.y(), output.positionWS.z());

	// doesn't support affine transformation, so doesn't need inverse transpose matrix
	output.normalWS = cb.objectToWorld.topLeftCorner(3, 3) * input.normalOS;
	output.positionCS = cb.projection * cb.worldToCamera * output.positionWS;
	return output;
}

Vector3f BlinnPhongFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input)
{
	//return Vector3f();
	const DirectionalLight& mainLight = cb.mainLight;
	Vector3f lightDir = mainLight.direction.normalized();
	Vector3f viewDir = (cb.cameraPosition - input.positionWS.head(3)).normalized();

	Vector3f H = (lightDir + viewDir).normalized();
	float NdotH = input.normalWS.dot(H);
	float NdotL = input.normalWS.dot(lightDir);
	NdotH = std::max(0.0f, NdotH);
	NdotL = std::max(0.0f, NdotL);

	Vector3f attenuatedLightColor = mainLight.color * (mainLight.distanceAttenuation * mainLight.shadowAttenuation) * NdotL;

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


template<typename T>
T lerp(const T& a, const T& b, float s) {
	return (1.0 - s) * a + s * b;
}

float saturate(float a) {
	return std::min(std::max(0.0f, a), 1.0f);
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

	float NdotL = saturate(input.normalWS.dot(cb.mainLight.direction));
	Vector3f radiance = cb.mainLight.color * cb.mainLight.distanceAttenuation * cb.mainLight.shadowAttenuation * NdotL;
	Vector3f brdf = brdfData.diffuse;

	Vector3f viewDir = (cb.cameraPosition - input.positionWS.head(3)).normalized();
	brdf += brdfData.specular * DirectBRDFSpecular(brdfData, input.normalWS, cb.mainLight.direction, viewDir);


	float temp = DirectBRDFSpecular(brdfData, input.normalWS, cb.mainLight.direction, viewDir);
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
	return output;
}

Vector3f ShadowMapFragmentShader(const ConstantBuffer& cb, const ShaderProperties& sp, const FragmentInput& input)
{
	return Vector3f(input.positionCS.z(), 0, 0);
}


