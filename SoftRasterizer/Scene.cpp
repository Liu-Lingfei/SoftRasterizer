#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "Scene.h"
#include "Shader.h"

Scene::Scene()
{
	Vector2f seed(rand() / double(RAND_MAX), rand() / double(RAND_MAX));
	//poissonDiskSamples(seed, 2, 10);
	uniformDiskSamples(seed);
	setSoftShadow(true);
	//setSoftShadow(false);

	const int shadowWidth = 1024;
	setShadowMapWidth(shadowWidth);

	int pixelNum = imageWidth * imageHeight;
	shadowMap = new float[shadowWidth * shadowWidth];
	depthBuffer = new float[pixelNum];
	colorBuffer = new uchar[pixelNum * 4];

	loadModel("./models/spot/spot_triangulated_good.obj");

	// reset camera
	resetModelMatrix();

	resetCamera();
	mainCamera.updateTransform();
	mainCamera.updateProjection();

	ConstantBuffer constantBuffer;
	mainCamera.createRenderer();


	// reset light

	resetLight();
	mainLight.updateCameraPose();
	mainLight.setShadowArea(0.1, 50, -2, 2, -2, 2);
	mainLight.createRenderer();


	//lightRenderer->bindAll(shadowWidth, shadowWidth, reader, )

	//cb.mainLight = mainLight;

	globalMaterial.shaderProps.albedo = Vector3f(0.9, 0.5, 0.5);
	globalMaterial.shaderProps.specular = Vector3f(0.1, 0.1, 0.1);
	globalMaterial.shaderProps.ambientColor = Vector3f(0.1, 0.1, 0.1);

	globalMaterial.shaderProps.smoothness = 0.8;
	globalMaterial.shaderProps.metallic = 0.8;

	globalMaterial.vertexShader = CommonVertexShader;
	globalMaterial.fragmentShader = UnityPBRFragmentShader;
	//globalMaterial.fragmentShader = BlinnPhongFragmentShader;

	shadowMaterial.vertexShader = ShadowMapVertexShader;
	shadowMaterial.fragmentShader = ShadowMapFragmentShader;

	//resetShadowCamera();


	//shadowCam.updateProjection();
	//shadowCam.updateTransform();
	//updateShadowConstantBuffer();

	//updateConstantBuffer();
}

Scene::~Scene()
{
	mainLight.disposeRenderer();
	mainCamera.disposeRenderer();
}

void Scene::render(uchar* buffer) {
	//updateConstantBuffer();

	const int shadowWidth = 1024;
	int pixelNum = imageWidth * imageHeight;

	//float* shadowMap = new float[shadowWidth * shadowWidth];
	//float* depthBuffer = new float[pixelNum];
	//uchar* colorBuffer = new uchar[pixelNum * 4];

	ConstantBuffer cameraCb;
	ConstantBuffer lightCb;

	setConstantBuffer(&lightCb, nullptr, mainLight.camera, &modelMatrix, &shadowMaterial);
	mainLight.bindReader(&reader);
	mainLight.bindDepthBuffer(shadowMap, shadowWidth);
	//mainLight.bindConstantBuffer(&lightCb);
	mainLight.bindConstantBuffer(lightCb);
	mainLight.camera->bindData(&triangles, &vertices, &normals, &texCoords, &colors, &completeTris);

	setConstantBuffer(&cameraCb, &mainLight, &mainCamera, &modelMatrix, &globalMaterial);
	mainCamera.bindReader(&reader);
	mainCamera.bindData(&triangles, &vertices, &normals, &texCoords, &colors, &completeTris);
	mainCamera.bindColorBuffer(colorBuffer, imageWidth, imageHeight);
	mainCamera.bindDepthBuffer(depthBuffer, imageWidth, imageHeight);
	mainCamera.bindShadowMap(shadowMap, shadowWidth);
	//mainCamera.bindConstantBuffer(&cameraCb);
	mainCamera.bindConstantBuffer(cameraCb);


	//std::thread lightThread(&Camera::renderDepthBuffer, mainLight.camera);
	//QFuture<void> fut1 = QtConcurrent::run(
	//	[&]() {
	//		mainLight.camera->renderDepthBuffer();
	//	}
	//);

	mainLight.camera->renderDepthBuffer();

	//std::vector<uchar> temp(pixelNum*4);
	//for (int i = 0; i < shadowWidth; ++i) {
	//	for (int j = 0; j < shadowWidth; ++j) {
	//	 int index = i + j * imageWidth;
	//	 int pixelIndex = i + j * shadowWidth;
	//	 temp[4 * index] = uchar(shadowMap[pixelIndex] * 255.99f);
	//		//qDebug("shadowMap[%d] = %f", pixelIndex, shadowMap[pixelIndex]);
	//	 temp[4 * index + 3] = 255;
	//	}
	//}
	//memcpy(buffer, temp.data(), pixelNum*4);

	//delete[] shadowMap;
	//delete[] depthBuffer;
	//delete[] colorBuffer;
	//return;


	//std::thread cameraThread(&Camera::renderDepthBuffer, &mainCamera);

	//lightThread.join();
	//cameraThread.join();

	//QFuture<void> fut2 = QtConcurrent::run(
	//	[&]() {
	//		mainCamera.renderDepthBuffer();
	//	}
	//);
	//fut1.waitForFinished();
	//fut2.waitForFinished();

	//mainCamera.renderDepthBuffer();

	//int x = 960;
	//int y = imageHeight - 1 - 500;
	//int index = x + y * imageWidth;
	//qDebug("depthBuffer[%d, %d] = %f", x, y, depthBuffer[index]);
	mainCamera.renderColorBuffer();

	memcpy(buffer, mainCamera.getColorBuffer(), pixelNum * 4);
}

void Scene::resetCamera()
{
	mainCamera.setPosition(Vector3f(0, 0, -4.0));
	mainCamera.setTarget(Vector3f(0, 0, 0));
	mainCamera.setUp(Vector3f(0, 1, 0));

	mainCamera.setVFov(90);
	mainCamera.setRatio(float(imageWidth) / float(imageHeight));
	mainCamera.setNear(0.1);
	mainCamera.setFar(1000);
}

void Scene::resetLight()
{
	mainLight.color = Vector3f(1, 1, 1);
	//mainLight.direction = Vector3f(0, 1, 0);
	//mainLight.position = Vector3f(0, 10, 0);
	mainLight.direction = Vector3f(1, 1, 0);
	mainLight.position = Vector3f(10, 10, 0);
}

void Scene::resetModelMatrix()
{
	modelMatrix = Matrix4f::Identity();
}

void Scene::loadModel(const std::string& filename)
{
	//std::string inputfile = "./models/spot/spot_triangulated_good.obj";
	//std::string inputfile = "./models/bunny/bunny.obj";
	tinyobj::ObjReaderConfig reader_config;
	reader_config.mtl_search_path = "./models/spot/"; // Path to material files
	//reader_config.mtl_search_path = "./models/bunny/"; // Path to material files

	//tinyobj::ObjReader reader;

	if (!reader.ParseFromFile(filename, reader_config)) {
		if (!reader.Error().empty()) {
			//std::cerr << "TinyObjReader: " << reader.Error();
			qDebug() << "TinyObjReader: " << QString::fromStdString(reader.Error());
		}
		exit(1);
	}

	if (!reader.Warning().empty()) {
		//std::cout << "TinyObjReader: " << reader.Warning();
		qDebug() << "TinyObjReader: " << QString::fromStdString(reader.Warning());
	}

	auto& attrib = reader.GetAttrib();
	auto& shapes = reader.GetShapes();
	auto& materials = reader.GetMaterials();

	int numVertices = attrib.vertices.size() / 3;
	int numTriangles = shapes[0].mesh.num_face_vertices.size();
	triangles.resize(numTriangles);
	vertices.resize(numVertices);
	normals.resize(numVertices);
	texCoords.resize(numVertices);
	colors.resize(numVertices);
	completeTris.resize(numTriangles);

	for (int i = 0; i < numTriangles; ++i) {
		triangles[i].x() = shapes[0].mesh.indices[3*i + 0].vertex_index;
		triangles[i].y() = shapes[0].mesh.indices[3*i + 1].vertex_index;
		triangles[i].z() = shapes[0].mesh.indices[3*i + 2].vertex_index;
	}

	float miny = 100;
	for (int i = 0; i < numVertices; ++i) {
		vertices[i].x() = attrib.vertices[3 * i + 0];
		vertices[i].y() = attrib.vertices[3 * i + 1];
		vertices[i].z() = attrib.vertices[3 * i + 2];
		miny = std::min(miny, vertices[i].y());

		normals[i].x() = attrib.normals[3 * i + 0];
		normals[i].y() = attrib.normals[3 * i + 1];
		normals[i].z() = attrib.normals[3 * i + 2];

		texCoords[i].x() = attrib.texcoords[2 * i + 0];
		texCoords[i].y() = attrib.texcoords[2 * i + 1];

		colors[i].x() = attrib.colors[3 * i + 0];
		colors[i].y() = attrib.colors[3 * i + 1];
		colors[i].z() = attrib.colors[3 * i + 2];
	}
	qDebug("miny = %f", miny);

	for (int i = 0; i < triangles.size(); ++i) {
		Vector3i indices = triangles[i];
		Triangle tri;
		int x = indices.x();
		int y = indices.y();
		int z = indices.z();

		Vector3f a = vertices[x];
		Vector3f b = vertices[y];
		Vector3f c = vertices[z];

		tri.v[0] = Vector4f(a.x(), a.y(), a.z(), 1.0);
		tri.v[1] = Vector4f(b.x(), b.y(), b.z(), 1.0);
		tri.v[2] = Vector4f(c.x(), c.y(), c.z(), 1.0);

		tri.normals[0] = normals[x];
		tri.normals[1] = normals[y];
		tri.normals[2] = normals[z];


		Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
		Vector3f aveNormal = (normals[x] + normals[y] + normals[z]).normalized();

		if (crossNormal.dot(aveNormal) < 0) {
			std::swap(triangles[i].x(), triangles[i].z());
			std::swap(tri.v[0], tri.v[2]);
			std::swap(tri.normals[0], tri.normals[2]);
		}

		completeTris[i] = tri;
	}
}

void Scene::setClearColor(const Vector4f& color)
{
	int pixIndex = 0;
	clearColor = color;
	for (int i = 0; i < imageWidth; ++i) {
		for (int j = 0; j < imageHeight; ++j) {
			clearBuffer[pixIndex + 0] = clearColor.x();
			clearBuffer[pixIndex + 1] = clearColor.y();
			clearBuffer[pixIndex + 2] = clearColor.z();
			clearBuffer[pixIndex + 3] = clearColor.w();
			pixIndex += 4;
		}
	}
}

void Scene::updateCameraVfov(float step) {
	float fov = mainCamera.vfov;
	fov -= step;
	fov = std::max(fov, 0.0f);
	fov = std::min(fov, 180.0f);
	mainCamera.setVFov(fov);
	mainCamera.updateProjection();
}

void Scene::updateCameraRotation(Vector3f rotationAxis, float rotationRadian) {
	mainCamera.rotate(rotationAxis, -rotationRadian);
	//cam.rotateAroundY(rotationRadians.x());
	//cam.rotateAroundX(rotationRadians.y());
	mainCamera.updateTransform();
}

void Scene::updateCameraPosition(const Vector3f& offset) {
	//qDebug("camera.z = (%f, %f, %f)", mainCamera.z.x(), mainCamera.z.y(), mainCamera.z.z());
	Vector3f directOffset = offset.x() * mainCamera.x + offset.y() * mainCamera.y + offset.z() * mainCamera.z;
	mainCamera.setPosition(mainCamera.pos + directOffset);
	mainCamera.setTarget(mainCamera.target + directOffset);
	mainCamera.updateTransform();
}


// need to be called each frame

void Scene::updateObjectToWorld(const Matrix3f& rotationMatrix) {
	modelMatrix.topLeftCorner(3, 3) = rotationMatrix * modelMatrix.topLeftCorner(3, 3);
}

//void Scene::updateConstantBuffer() {
//	cb.lightPos = mainLight.position;
//	cb.lightDir = mainLight.direction.normalized();
//	cb.lightCol = mainLight.color;
//
//	cb.lightShadowViewMatrix = mainLight.cam->getWorldToCamera();
//	cb.lightShadowProjMatrix = mainLight.cam->getProjection();
//	cb.lightShadowMap = mainLight.depthBuffer;
//	cb.lightShadowWidth = mainLight.depthBufferWidth;
//
//	cb.cameraPosition = mainCamera.pos;
//	cb.worldToCamera = mainCamera.worldToCamera;
//	cb.projection = mainCamera.projection;
//	cb.cameraType = mainCamera.type;
//
//	cb.material = &globalMaterial;
//	cb.objectToWorld = modelMatrix;
//	//cb.setPerLightInfo(mainLight);
//	//cb.setPerCameraInfo(mainCamera);
//	//cb.setMaterial(&globalMaterial);
//	//cb.setObjectToWorld(modelMatrix);
//}

void Scene::setConstantBuffer(ConstantBuffer* buffer, const DirectionalLight* light, const Camera* camera, const Matrix4f* objectToWorld, Material* material)
{
	if (light) {
		buffer->lightPos = light->position;
		buffer->lightDir = light->direction.normalized();
		buffer->lightCol = light->color;

		buffer->lightShadowViewMatrix = light->camera->getWorldToCamera();
		buffer->lightShadowProjMatrix = light->camera->getProjection();
		buffer->lightShadowMap = light->camera->getDepthBuffer();
		buffer->lightShadowWidth = light->camera->getWidth();
		//buffer->lightShadowMap = light->depthBuffer;
		//buffer->lightShadowWidth = light->depthBufferWidth;
	}

	if (camera) {
		buffer->cameraPosition = camera->pos;
		buffer->worldToCamera = camera->worldToCamera;
		buffer->projection = camera->projection;
		buffer->cameraType = camera->type;
	}

	if (objectToWorld) {
		buffer->objectToWorld = *objectToWorld;
	}
	buffer->material = material;
}

void Scene::update() {

}
