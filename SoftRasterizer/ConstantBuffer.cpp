//#include "ConstantBuffer.h"
//
//void ConstantBuffer::setObjectToWorld(const Matrix4f& m) { objectToWorld = m; }
//
//void ConstantBuffer::setMaterial(Material* m) { material = m; }
//
//void ConstantBuffer::setPerCameraInfo(const Camera& cam) {
//	cameraPosition = cam.pos;
//	worldToCamera = cam.worldToCamera;
//	projection = cam.projection;
//	cameraType = cam.type;
//}
//
//void ConstantBuffer::setPerLightInfo(DirectionalLight& l) {
//	lightPos = l.position;
//	lightDir = l.direction.normalized();
//	lightCol = l.color;
//
//	lightShadowViewMatrix = l.cam->getWorldToCamera();
//	lightShadowProjMatrix = l.cam->getProjection();
//	lightShadowMap = l.depthBuffer;
//	lightShadowWidth = l.depthBufferWidth;
//}