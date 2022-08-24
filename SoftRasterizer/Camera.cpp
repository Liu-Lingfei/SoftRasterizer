#include "Camera.h"

//Camera::Camera(int type, const Vector3f& p, const Vector3f& t, const Vector3f& u)
//	:
//	type(type),
//	pos(p), target(t), up(u),
//	x(Vector3f::Zero()), y(Vector3f::Zero()), z(Vector3f::Zero()),
//	near(0), far(0),
//	worldToCamera(Matrix4f::Identity()),
//	cameraToWorld(Matrix4f::Identity()),
//	projection(Matrix4f::Zero()),
//	renderer(nullptr),
//	shadowMap(nullptr),
//	depthBuffer(nullptr),
//	colorBuffer(nullptr)
//{
//}


//Camera::Camera(int type, const Vector3f& p, const Vector3f& t, const Vector3f& u)
//	:
//	type(type),
//	pos(p), target(t), up(u),
//	x(Vector3f::Zero()), y(Vector3f::Zero()), z(Vector3f::Zero()),
//	near(0), far(0),
//	worldToCamera(Matrix4f::Identity()),
//	cameraToWorld(Matrix4f::Identity()),
//	projection(Matrix4f::Zero()),
//	renderer(nullptr),
//	shadowMap(nullptr),
//	depthBuffer(nullptr),
//	colorBuffer(nullptr)
//{
//}

//Camera::Camera(int type, const Vector3f& p, const Vector3f& t, const Vector3f& u)
//{
//}

void Camera::renderColorBuffer()
{
	renderer->renderColorBuffer();
}

void Camera::renderDepthBuffer()
{
	renderer->renderDepthBuffer();
}

int Camera::getWidth()
{
	return renderer->screenWidth;
}

int Camera::getHeight()
{
	return renderer->screenHeight;
}

const float* Camera::getDepthBuffer() const
{
	return renderer->depthBuffer;
}

const float* Camera::getShadowMap() const
{
	return renderer->shadowMap;
}

const uchar* Camera::getColorBuffer() const
{
	return renderer->colorBuffer;
}

void Camera::setPosition(const Vector3f& p)
{
	pos = p;
}

void Camera::setTarget(const Vector3f& t)
{
	target = t;
}

void Camera::setUp(const Vector3f& u)
{
	up = u;
}

void Camera::createRenderer()
{
	if (renderer) {
		disposeRenderer();
	}
	renderer = new FastRenderer();
}

void Camera::disposeRenderer()
{
	delete renderer;
	renderer = nullptr;
}

//void Camera::bindRenderer(FastRenderer* r) {
	//renderer = r;
//}

void Camera::bindShadowMap(float* sm, int width)
{
	renderer->bindShadowMap(sm, width);
}

void Camera::disposeShadowMap()
{
	renderer->disposeShadowMap();
}

void Camera::bindDepthBuffer(float* db, int width, int height)
{
	renderer->bindDepthBuffer(db, width, height);
}

void Camera::disposeDepthBuffer()
{
	renderer->disposeDepthBuffer();
}

void Camera::bindColorBuffer(uchar* cb, int width, int height)
{
	renderer->bindColorBuffer(cb, width, height);
}

void Camera::disposeColorBuffer()
{
	renderer->disposeColorBuffer();
}

//void Camera::bindConstantBuffer(ConstantBuffer* cb) {
void Camera::bindConstantBuffer(const ConstantBuffer& cb) {
	renderer->bindConstantBuffer(cb);
}

void Camera::disposeConstantBuffer()
{
	renderer->disposeConstantBuffer();
}

void Camera::bindReader(const tinyobj::ObjReader* r)
{
	renderer->bindReader(r);
}

void Camera::disposeReader()
{
	renderer->disposeReader();
}

void Camera::bindData(
	const std::vector<Vector3i>* triangles,
	const std::vector<Vector3f>* vertices,
	const std::vector<Vector3f>* normals,
	const std::vector<Vector2f>* texCoords,
	const std::vector<Vector3f>* colors,
	const std::vector<Triangle>* completeTris
)
{
	renderer->bindData(triangles, vertices, normals, texCoords, colors, completeTris);
}

void Camera::setSize(int width, int height)
{
	renderer->setSize(width, height);
}




Matrix4f Camera::getWorldToCamera() const
{
	return worldToCamera;
}

Matrix4f Camera::getCameraToWorld() const
{
	return cameraToWorld;
}

Matrix4f Camera::getProjection() const
{
	return projection;
}

void Camera::rotate(Vector3f axis, float radian)
{
	Vector3f viewDir = target - pos;
	axis.normalize();
	Eigen::AngleAxisf rotationVector(radian, axis);

	Matrix3f rotationMatrix = rotationVector.matrix();
	worldToCamera.topLeftCorner(3, 3) = rotationMatrix * worldToCamera.topLeftCorner(3, 3);

	//Vector3f newViewDir = rotationMatrix * viewDir;
	//up = rotationMatrix * up;
	Vector3f newViewDir = worldToCamera.block(2, 0, 1, 3).transpose();
	target = pos + newViewDir;
	up = worldToCamera.block(1, 0, 1, 3).transpose();
}

void Camera::rotateAroundX(float radian)
{
	Vector3f axis(1, 0, 0);
	Vector3f viewDir = target - pos;
	axis.normalize();
	Eigen::AngleAxisf rotationVector(radian, axis);
	Matrix3f rotationMatrix = rotationVector.matrix();
	Vector3f newViewDir = rotationMatrix * viewDir;
	up = rotationMatrix * up;
	target = pos + newViewDir;
}

void Camera::rotateAroundY(float radian)
{
	Vector3f axis(0, 1, 0);
	Vector3f viewDir = target - pos;
	axis.normalize();
	Eigen::AngleAxisf rotationVector(radian, axis);
	Matrix3f rotationMatrix = rotationVector.matrix();
	Vector3f newViewDir = rotationMatrix * viewDir;
	up = rotationMatrix * up;
	target = pos + newViewDir;
}

void Camera::updateTransform()
{
	z = (target - pos).normalized();
	x = (up.cross(z)).normalized();
	y = z.cross(x);

	worldToCamera(0, 0) = x.x();
	worldToCamera(0, 1) = x.y();
	worldToCamera(0, 2) = x.z();

	worldToCamera(1, 0) = y.x();
	worldToCamera(1, 1) = y.y();
	worldToCamera(1, 2) = y.z();

	worldToCamera(2, 0) = z.x();
	worldToCamera(2, 1) = z.y();
	worldToCamera(2, 2) = z.z();

	worldToCamera(0, 3) = -x.dot(pos);
	worldToCamera(1, 3) = -y.dot(pos);
	worldToCamera(2, 3) = -z.dot(pos);


	cameraToWorld.topLeftCorner(3, 3) = worldToCamera.topLeftCorner(3, 3).transpose();
	cameraToWorld.topRightCorner(3, 1) = pos;
}

void Camera::updateProjection() {
	if (type == 0) updatePerspectiveProjection();
	else if (type == 1) updateOrthographicProjection();
	else return;
}

void Camera::updatePerspectiveProjection()
{
	projection = Matrix4f::Zero();
	float halfRadianFov = vfov * M_PI * 0.5 / 180.0;
	float c = 1.0 / std::tan(halfRadianFov);
	//qDebug("helfRadianFov = %f", halfRadianFov);
	//qDebug("c = %f", c);

	projection(0, 0) = c / ratio;
	projection(1, 1) = c;
	projection(2, 2) = -near / (far - near);
	projection(2, 3) = near * far / (far - near);
	projection(3, 2) = 1;
}

void Camera::updateOrthographicProjection()
{
	//float halfRadianFov = vfov * M_PI * 0.5 / 180.0;
	//float c = 1.0 / std::tan(halfRadianFov);
	//c = c / near;
	//projection(0, 0) = c / ratio;
	//projection(1, 1) = c;
	projection = Matrix4f::Identity();

	projection(0, 0) = 2.0f / (right - left);
	projection(1, 1) = 2.0f / (top - bottom);

	projection(2, 2) = -1 / (far - near);
	projection(2, 3) = far / (far - near);
	projection(3, 3) = 1;

	//qDebug("(left, right) = (%f, %f)", left, right);
	//qDebug("(bottom, top) = (%f, %f)", bottom, top);

	//MyTransform::print(projection);
	//qDebug("t - b = %f", 2 / c);
	//qDebug("r - l = %f", 2 / (c/ratio));
	//qDebug("f - n = %f", (far - near));
}

void Camera::setNear(float n) { near = n; }

void Camera::setFar(float f) { far = f; }

void Camera::setVFov(float f) { vfov = f; }

void Camera::setRatio(float r) { ratio = r; }

void Camera::setLeft(float l) { left = l; }

void Camera::setRight(float r) { right = r; }

void Camera::setBottom(float b) { bottom = b; }

void Camera::setTop(float t) { top = t; }

