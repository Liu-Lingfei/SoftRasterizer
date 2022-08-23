#include "Light.h"

DirectionalLight::DirectionalLight(const Vector3f& p, const Vector3f& d, const Vector3f c)
	: position(p), direction(d), color(c),
	//depthBufferWidth(0),
	//depthBuffer(nullptr),
	camera(new Camera(1, p, p - d, Vector3f(0, 1, 0)))
{
}

DirectionalLight::~DirectionalLight()
{
	delete camera;
}

void DirectionalLight::setShadowArea(float near, float far, float left, float right, float bottom, float top)
{
	camera->setNear(near);
	camera->setFar(far);
	camera->setLeft(left);
	camera->setRight(right);
	camera->setBottom(bottom);
	camera->setTop(top);
	camera->updateProjection();
}

void DirectionalLight::updateCameraPose()
{
	camera->pos = position;
	camera->target = position - direction;
	camera->up = Vector3f(-direction.y(), direction.x(), 0);
	camera->updateTransform();
}

void DirectionalLight::createRenderer()
{
	camera->createRenderer();
}

void DirectionalLight::disposeRenderer()
{
	camera->disposeRenderer();
}

void DirectionalLight::bindReader(tinyobj::ObjReader* reader)
{
	camera->bindReader(reader);
}

void DirectionalLight::disposeReader()
{
	camera->disposeReader();
}

void DirectionalLight::bindDepthBuffer(float* db, int w)
{
	//depthBufferWidth = w;
	//depthBuffer = db;
	camera->bindDepthBuffer(db, w, w);
}

void DirectionalLight::disposeDepthBuffer()
{
	camera->disposeDepthBuffer();
}

//void DirectionalLight::bindConstantBuffer(ConstantBuffer* cb)
void DirectionalLight::bindConstantBuffer(const ConstantBuffer& cb)
{
	camera->bindConstantBuffer(cb);
}

void DirectionalLight::disposeConstantBuffer()
{
	camera->disposeConstantBuffer();
}
