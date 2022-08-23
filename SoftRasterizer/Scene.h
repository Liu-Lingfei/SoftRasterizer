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
	Scene();

	~Scene();

	void render(uchar* buffer);

	void resetCamera();

	void resetLight();

	void resetModelMatrix();

	void loadModel(const std::string& filename);

	void setClearColor(const Vector4f& color);

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


	void update();

	const int imageWidth = 1920;
	const int imageHeight = 1080;

	Matrix4f modelMatrix;
	tinyobj::ObjReader reader;
	Camera mainCamera;
	DirectionalLight mainLight;
	//ConstantBuffer cb;
	Material globalMaterial;
	Material shadowMaterial;

	Vector4f clearColor;
	std::vector<uchar> clearBuffer;

	float* depthBuffer;
	float* shadowMap;
	uchar* colorBuffer;

	std::vector<Vector3i> triangles;
	std::vector<Vector3f> vertices;
	std::vector<Vector3f> normals;
	std::vector<Vector2f> texCoords;
	std::vector<Vector3f> colors;
	std::vector<Triangle> completeTris;
};

#endif // !SCENE_H
