#ifndef CAMERA_H
#define CAMERA_H

#include "Eigen/Core"
#include "FastRenderer.h"

using namespace Eigen;

class FastRenderer;
struct ConstantBuffer;
//typedef unsigned char uchar;

class Camera
{
public:

	Camera(int type=0, const Vector3f& p=Vector3f::Zero(), const Vector3f& t=Vector3f(0, 0, 1), const Vector3f& u=Vector3f(0, 1, 0)) :
		type(type),
		pos(p), target(t), up(u),
		near(1), far(1000),
		left(-1), right(1), bottom(-1), top(1),
		vfov(90), ratio(1),
		x(Vector3f::Zero()), y(Vector3f::Zero()), z(Vector3f::Zero()),
		worldToCamera(Matrix4f::Identity()),
		cameraToWorld(Matrix4f::Identity()),
		projection(Matrix4f::Zero()),
		renderer(nullptr)
		//shadowMap(nullptr),
		//depthBuffer(nullptr),
		//colorBuffer(nullptr),
		//constantBuffer(nullptr)
	{
		updateTransform();
		updateProjection();
	}

	//Camera(int type = 0, const Vector3f& p = Vector3f::Zero(), const Vector3f& t = Vector3f(0, 0, 1), const Vector3f& u = Vector3f(0, 1, 0)) :
	//	type(type),
	//	pos(p), target(t), up(u),
	//	near(1), far(1000),
	//	left(-1), right(1), bottom(-1), top(1),
	//	vfov(90), ratio(1),
	//	//x(Vector3f::Zero()), y(Vector3f::Zero()), z(Vector3f::Zero()),
	//	//worldToCamera(Matrix4f::Identity()),
	//	//cameraToWorld(Matrix4f::Identity()),
	//	//projection(Matrix4f::Zero()),
	//	renderer(nullptr)
	//	//shadowMap(nullptr),
	//	//depthBuffer(nullptr),
	//	//colorBuffer(nullptr),
	//	//constantBuffer(nullptr)
	//{
	//	updateTransform();
	//	updateProjection();
	//}

	void renderColorBuffer();
	void renderDepthBuffer();

	int getWidth();
	int getHeight();

	float* getDepthBuffer();
	float* getShadowMap();
	uchar* getColorBuffer();

	void setPosition(const Vector3f& p);
	void setTarget(const Vector3f& t);
	void setUp(const Vector3f& u);

	void createRenderer();
	void disposeRenderer();

	//void bindRenderer(FastRenderer* r);

	void bindShadowMap(float* sm, int width);
	void disposeShadowMap();

	void bindDepthBuffer(float* db, int width, int height);
	void disposeDepthBuffer();

	void bindColorBuffer(uchar* cb, int width, int height);
	void disposeColorBuffer();

	//void bindConstantBuffer(ConstantBuffer* cb);
	void bindConstantBuffer(const ConstantBuffer& cb);
	void disposeConstantBuffer();

	void bindReader(const tinyobj::ObjReader* r);
	void disposeReader();

	void bindData(
		const std::vector<Vector3i>* triangles,
		const std::vector<Vector3f>* vertices,
		const std::vector<Vector3f>* normals,
		const std::vector<Vector2f>* texCoords,
		const std::vector<Vector3f>* colors,
		const std::vector<Triangle>* completeTris
	);
	void disposeData();

	void setSize(int width, int height);

	void updateTransform();
	void updateProjection();
	void updatePerspectiveProjection();
	void updateOrthographicProjection();

	void setNear(float n);
	void setFar(float f);

	void setVFov(float f);;
	void setRatio(float r);;

	void setLeft(float l);
	void setRight(float r);
	void setBottom(float b);
	void setTop(float t);

	int type;
	Matrix4f getWorldToCamera() const;
	Matrix4f getCameraToWorld() const;
	Matrix4f getProjection() const;

	void rotate(Vector3f axis, float radian);
	void rotateAroundX(float radian);
	void rotateAroundY(float radian);

	Vector3f pos;
	Vector3f target;
	Vector3f up;

	Vector3f x, y, z;

	float near;
	float far;

	// for perspective camera

	// in degrees, not radians
	float vfov;
	// screenWidth/screenHeight
	float ratio;

	// for orthographic camera

	float left, right, bottom, top;


	Matrix4f worldToCamera;
	Matrix4f cameraToWorld;
	Matrix4f projection;

	FastRenderer* renderer;

	//float* shadowMap;
	//float* depthBuffer;
	//uchar* colorBuffer;
	//ConstantBuffer* constantBuffer;
	//const tinyobj::ObjReader* reader = nullptr;
};

#endif // !CAMERA_H