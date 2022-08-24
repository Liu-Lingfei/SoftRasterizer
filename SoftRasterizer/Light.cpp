#include "Light.h"

DirectionalLight::DirectionalLight(int type, const Vector3f& p, const Vector3f& d, const Vector3f c):
	lightType(type), position(p), direction(d), color(c),
	camera(new Camera(1 - type, p, p - d, Vector3f(0, 0, 1)))
{
}

DirectionalLight::~DirectionalLight()
{
	delete camera;
}

void DirectionalLight::setDirectionalLightShadowArea(float near, float far, float left, float right, float bottom, float top)
{
	camera->setNear(near);
	camera->setFar(far);
	camera->setLeft(left);
	camera->setRight(right);
	camera->setBottom(bottom);
	camera->setTop(top);
	camera->updateProjection();
}

void DirectionalLight::setPointLightLightShadowArea(float near, float far, float vfov, float ratio)
{
	camera->setNear(near);
	camera->setFar(far);
	camera->setVFov(vfov);
	camera->setRatio(ratio);
	camera->updateProjection();
}

void DirectionalLight::updateCameraPose()
{
	camera->pos = position;
	camera->target = position - direction;

	if (direction.x() == 0 && direction.y() == 0) {
		camera->up = Vector3f(0, 1, 0);
	}
	else {
		float y = std::abs(direction.x());
		float x = direction.x() > 0 ? -direction.y() : direction.y();

		camera->up = Vector3f(x, y, 0);
	}
	camera->updateTransform();
}

void DirectionalLight::toPointLight()
{
	if (lightType == 0) {
		lightType = 1;
		if (camera) {
			float near = camera->near;
			float far = camera->far;
			float right = camera->right;
			float top = camera->top;

			float ratio = right / top;
			float vfov = std::atan(top / near) * 180 / M_PI;

			Vector3f up = camera->up;
			//qDebug("up = (%f, %f, %f)", up.x(), up.y(), up.z());
			//qDebug("direction = (%f, %f, %f)", direction.x(), direction.y(), direction.z());
			//qDebug("position = (%f, %f, %f)", position.x(), position.y(), position.z());

			qDebug("before to point light, transform = ");
			MyTransform::print(camera->getWorldToCamera());
			qDebug("before to point light, projection = ");
			MyTransform::print(camera->getProjection());

			delete camera;
			camera = new Camera(0, position, position - direction, up);
			//camera = new Camera();
			camera->setNear(near);
			camera->setFar(far);
			camera->setVFov(vfov);
			camera->setRatio(ratio);
			camera->updateTransform();
			camera->updateProjection();

			qDebug("after to point light, transform = ");
			MyTransform::print(camera->getWorldToCamera());
			qDebug("after to point light, projection = ");
			MyTransform::print(camera->getProjection());
		}
	}
}

void DirectionalLight::toDirectionalLight()
{
	if (lightType == 1) {
		lightType = 0;
		if (camera) {
			float near = camera->near;
			float far = camera->far;
			float vfov = camera->vfov;
			float ratio = camera->ratio;

			float top = std::tan(vfov * M_PI / 180) * near;
			float right = top * ratio;

			Vector3f up = camera->up;

			qDebug("before to directional light, transform = ");
			MyTransform::print(camera->getWorldToCamera());
			qDebug("before to directional light, projection = ");
			MyTransform::print(camera->getProjection());

			delete camera;
			camera = new Camera(1, position, position - direction, up);
			camera->setNear(near);
			camera->setFar(far);
			camera->setLeft(-right);
			camera->setRight(right);
			camera->setBottom(-top);
			camera->setTop(top);
			camera->updateTransform();
			camera->updateProjection();

			qDebug("after to directional light, transform = ");
			MyTransform::print(camera->getWorldToCamera());
			qDebug("after to directional light, projection = ");
			MyTransform::print(camera->getProjection());
		}
	}
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
