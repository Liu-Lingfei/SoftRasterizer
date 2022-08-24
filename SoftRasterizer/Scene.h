#ifndef SCENE_H
#define SCENE_H

#include "Light.h"
#include "Camera.h"
#include "ConstantBuffer.h"

#include <thread>
#include <iostream>

#include <QtConcurrent/QtConcurrent>
using namespace QtConcurrent;

struct Scene
{
	Scene(int w, int h, int sw);

	~Scene();

	void render(uchar* buffer, uchar* dBuffer, int imageWidth, int imageHeight, int shadowWidth);

	void resetCamera(int imageWidth, int imageHeight);

	void resetLight();

	void resetModelMatrix();

	void loadModel(const std::string& filename);

	void updateCameraVfov(float step);

	void updateCameraRotation(Vector3f rotationAxis, float rotationRadian);

	void updateCameraPosition(const Vector3f& offset);

	void updateObjectToWorld(const Matrix3f& rotationMatrix);

	//// need to be called each frame
	//void updateConstantBuffer();

	void setConstantBuffer(
		ConstantBuffer* buffer,
		const DirectionalLight* light,
		const Camera* camera,
		const Matrix4f* objectToWorld,
		Material* material
		);

	void setDisplayShadowMap(bool displayShadowMap) { this->displayShadowMap = displayShadowMap; }
	void setRed(int r) {globalMaterial.shaderProps.albedo.x() = (float)r / (float)255;}
	void setGreen(int g) {globalMaterial.shaderProps.albedo.y() = (float)g / (float)255;}
	void setBlue(int b) {globalMaterial.shaderProps.albedo.z() = (float)b / (float)255;}
	void setSmoothness(int s) {globalMaterial.shaderProps.smoothness = (float)s / (float)99;}
	void setMetallic(int m) {globalMaterial.shaderProps.metallic = (float)m / (float)99;}

	void setLightIntensity(int i) { float lightIntensity = i/10.0f; mainLight.color = lightIntensity * lightColor; }
	void setLightRed(int r) { lightColor.x() = (float)r / (float)255; mainLight.color.x() = lightColor.x() * lightIntensity; }
	void setLightGreen(int g) { lightColor.y() = (float)g / (float)255; mainLight.color.y() = lightColor.y() * lightIntensity; }
	void setLightBlue(int b) { lightColor.z() = (float)b / (float)255; mainLight.color.z() = lightColor.z() * lightIntensity; }

	void setDirectionalLight(bool checked) {
		qDebug("setDirectinalLight, checked = %d", checked);
		if (checked && mainLight.lightType == 1) {
			qDebug("main.lightType == 1");
			mainLight.toDirectionalLight();
			mainLight.setDirectionalLightShadowArea(1, 10, -2, 2, -2, 2);
		}
	}
	void setPointLight(bool checked) {
		qDebug("setPointLight, checked = %d", checked);
		if (checked && mainLight.lightType == 0) {
			qDebug("main.lightType == 0");
			mainLight.toPointLight();
			mainLight.setPointLightLightShadowArea(1, 1000, 90, 1);
		}
	}

	//int imageWidth;
	//int imageHeight;
	//int shadowWidth;

	float lightIntensity = 1;
	Vector3f lightColor = Vector3f(1, 1, 1);

	Matrix4f modelMatrix;
	tinyobj::ObjReader reader;
	Camera mainCamera;
	DirectionalLight mainLight;
	//ConstantBuffer cb;
	Material globalMaterial;
	Material shadowMaterial;

	Vector4f clearColor;
	std::vector<uchar> clearBuffer;

	//float* depthBuffer;
	//float* shadowMap;
	//uchar* colorBuffer;

	std::vector<Vector3i> triangles;
	std::vector<Vector3f> vertices;
	std::vector<Vector3f> normals;
	std::vector<Vector2f> texCoords;
	std::vector<Vector3f> colors;
	std::vector<Triangle> completeTris;

	bool displayShadowMap;
};

#endif // !SCENE_H
