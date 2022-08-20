#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "SoftRasterizer.h"

SoftRasterizer::SoftRasterizer(QWidget* parent)
    :
    QMainWindow(parent),
    pixelSize(imageWidth* imageHeight),
    bufferSize(imageWidth* imageHeight * 4),
    //clearBuffer(new uchar[bufferSize]),
    clearBuffer(bufferSize, 0),
    //depthBuffer(new float[pixelSize]),
    depthBuffer(pixelSize),
    shadowMaps(4),
    img(imageWidth, imageHeight, QImage::Format_RGBA8888),
    frontBuffer(img.bits()),
    cam(Vector3f(0, 0, -10), Vector3f(0, 0, 0), Vector3f(0, 1, 0)),
    lastFrameMousePos(0, 0),
    deltaTime(0),
    modelMatrix(Matrix4f::Identity())
    //V(Matrix4f::Identity()),
    //P(Matrix4f::Identity()),
    //MVP(Matrix4f::Identity())
    //vertexShader(BlinnPhongVertexShader),
    //fragmentShader(BlinnPhongFragmentShader)
{
    ui.setupUi(this);
    resize(1920, 1080);

    shadowMaps[0].resize(shadowWidth*shadowWidth, 0.0f);
    shadowMaps[1].resize((shadowWidth/2) * (shadowWidth/2), 0.0f);
    shadowMaps[2].resize((shadowWidth/4) * (shadowWidth/4), 0.0f);
    shadowMaps[3].resize((shadowWidth/8) * (shadowWidth/8), 0.0f);

    img.fill(Qt::white);
    label.setParent(this);
    label.setGeometry(0, 0, img.width(), img.height());
    label.show();

    //clearBuffer = new uchar[bufferSize];
    setClearColor(Vector4f(0, 0, 0, 255));

    //cam.setVFov(90);
    //cam.setRatio(float(imageWidth) / float(imageHeight));
    //cam.setNear(1);
    //cam.setFar(50);
    resetCamera();
    cam.updateTransform();
    cam.updateProjection();

    loadModel("");

    mainLight.color = Vector3f(1, 1, 1);
    mainLight.direction = Vector3f(0, 1, 0);
    mainLight.position = Vector3f(0, 40, 0);

    cb.mainLight = mainLight;
    updateConstantBuffer();

    m.shaderProps.albedo = Vector3f(0.9, 0.5, 0.5);
    m.shaderProps.specular = Vector3f(0.1, 0.1, 0.1);
    m.shaderProps.ambientColor = Vector3f(0.1, 0.1, 0.1);

    m.shaderProps.smoothness = 0.8;
    m.shaderProps.metallic = 0.8;

    m.vertexShader = CommonVertexShader;
    m.fragmentShader = UnityPBRFragmentShader;

    shadowMaterial.vertexShader = ShadowMapVertexShader;
    shadowMaterial.fragmentShader = ShadowMapFragmentShader;

    resetShadowCamera();
    shadowCam.updateProjection();
    shadowCam.updateTransform();
    updateShadowConstantBuffer();
}

SoftRasterizer::~SoftRasterizer()
{
    //delete[] clearBuffer;
    //delete[] depthBuffer;
}

int SoftRasterizer::drawTriangle(uchar* backBuffer, Triangle& tri)
{
    Triangle screenSpaceTri;

    Vector3f a = tri.v[0].head(3);
    Vector3f b = tri.v[1].head(3);
    Vector3f c = tri.v[2].head(3);

    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
    if (crossNormal.dot(aveNormal) < 0) {
        //qDebug() << "not counter clockwise";
        std::swap(tri.v[0], tri.v[2]);
        std::swap(tri.normals[0], tri.normals[2]);
        std::swap(tri.texCoords[0], tri.texCoords[2]);
        std::swap(tri.colors[0], tri.colors[2]);
    }

    FragmentInput fi[3];
    for (int i = 0; i < 3; ++i) {
		VertexInput vi;
		vi.positionOS = tri.v[i];
        vi.normalOS = tri.normals[i];
        fi[i] = m.vertexShader(cb, m.shaderProps, vi);
    }

    screenSpaceTri.v[0] = screenMapping(imageWidth, imageHeight, fi[0].positionCS);
    screenSpaceTri.normals[0] = fi[0].normalWS;

    screenSpaceTri.v[1] = screenMapping(imageWidth, imageHeight, fi[1].positionCS);
    screenSpaceTri.normals[1] = fi[1].normalWS;

    screenSpaceTri.v[2] = screenMapping(imageWidth, imageHeight, fi[2].positionCS);
    screenSpaceTri.normals[2] = fi[2].normalWS;

    bool clip = true;
    for (int i = 0; i < 3; ++i) {
        if (screenSpaceTri.v[i].x() >= 0 && screenSpaceTri.v[i].x() <= imageWidth &&
            screenSpaceTri.v[i].y() >= 0 && screenSpaceTri.v[i].y() <= imageHeight &&
            screenSpaceTri.v[i].z() >= 0 && screenSpaceTri.v[i].z() < 1
            ) {
            clip = false;
            break;
        }
    }

    if (clip) return 0;

    //Triangle rasterized;
    //rasterized.v[0] = screenMapping(MVP * tri.v[0]);
    //rasterized.v[1] = screenMapping(MVP * tri.v[1]);
    //rasterized.v[2] = screenMapping(MVP * tri.v[2]);


    //Vector4f cs0 = MVP * tri.v[0];
    //Vector4f cs1 = MVP * tri.v[1];
    //Vector4f cs2 = MVP * tri.v[2];
    ////assert(fi[0].positionCS == cs0);
    ////assert(fi[1].positionCS == cs1);
    ////assert(fi[2].positionCS == cs2);

    //Matrix3f model = M.topLeftCorner(3, 3);
    //rasterized.normals[0] = model * tri.normals[0];
    //rasterized.normals[1] = model * tri.normals[1];
    //rasterized.normals[2] = model * tri.normals[2];

    //assert(rasterized.normals[0] == screenSpaceTri.normals[0]);
    //assert(rasterized.normals[1] == screenSpaceTri.normals[1]);
    //assert(rasterized.normals[2] == screenSpaceTri.normals[2]);

    screenSpaceTri.updateInfo();
    //rasterized.updateInfo();

    float minx = imageWidth, miny = imageHeight;
    float maxx = 0, maxy = 0;
    for (int i = 0; i < 3; i++) {
        minx = screenSpaceTri.v[i].x() < minx ? screenSpaceTri.v[i].x() : minx;
        maxx = screenSpaceTri.v[i].x() > maxx ? screenSpaceTri.v[i].x() : maxx;

        miny = screenSpaceTri.v[i].y() < miny ? screenSpaceTri.v[i].y() : miny;
        maxy = screenSpaceTri.v[i].y() > maxy ? screenSpaceTri.v[i].y() : maxy;
    }

    //float minx = imageWidth, miny = imageHeight;
    //float maxx = 0, maxy = 0;
    //for (int i = 0; i < 3; i++) {
    //    minx = rasterized.v[i].x() < minx ? rasterized.v[i].x() : minx;
    //    maxx = rasterized.v[i].x() > maxx ? rasterized.v[i].x() : maxx;

    //    miny = rasterized.v[i].y() < miny ? rasterized.v[i].y() : miny;
    //    maxy = rasterized.v[i].y() > maxy ? rasterized.v[i].y() : maxy;
    //}

    int iminx = floor(minx);
    int imaxx = ceil(maxx);
    int iminy = floor(miny);
    int imaxy = ceil(maxy);

    iminx = std::max(0, iminx);
    imaxx = std::min(imageWidth - 1, imaxx);
    iminy = std::max(0, iminy);
    imaxy = std::min(imageHeight - 1, imaxy);

    //qDebug("iminx = %d, imaxx = %d, iminy = %d, imaxy = %d", iminx, imaxx, iminy, imaxy);

    //clock_t start, stop;
    //start = clock();
    int counter = 0;
    Vector3f bary(0, 0, 0);


    Vector3f equationResult = screenSpaceTri.precomputeLineEquations(iminx + 0.5, iminy + 0.5);
    for (int x = iminx; x <= imaxx; x++) {
        for (int y = iminy; y <= imaxy; y++) {
            int pixelIndex = x + (imageHeight - 1 - y) * imageWidth;
            int startIndex = pixelIndex*4;
            Vector3f currResult;
            int xoffset = x - iminx;
            int yoffset = y - iminy;
            currResult.x() = equationResult.x() + xoffset * screenSpaceTri.edgeCoeffs[0].x() + yoffset * screenSpaceTri.edgeCoeffs[0].y();
            currResult.y() = equationResult.y() + xoffset * screenSpaceTri.edgeCoeffs[1].x() + yoffset * screenSpaceTri.edgeCoeffs[1].y();
            currResult.z() = equationResult.z() + xoffset * screenSpaceTri.edgeCoeffs[2].x() + yoffset * screenSpaceTri.edgeCoeffs[2].y();
            if (screenSpaceTri.pointInsideTriangleFast(x+0.5, y+0.5, currResult, bary)) {
                float depth = bary.x() * screenSpaceTri.v[0].z() + bary.y() * screenSpaceTri.v[1].z() + bary.z() * screenSpaceTri.v[2].z();
                //float xCoord = bary.x() * screenSpaceTri.v[0].x() + bary.y() * screenSpaceTri.v[1].x() + bary.z() * screenSpaceTri.v[2].x();
                //float yCoord = bary.x() * screenSpaceTri.v[0].y() + bary.y() * screenSpaceTri.v[1].y() + bary.z() * screenSpaceTri.v[2].y();

                if (depth > 0 && depth < 1 && depth >= depthBuffer[pixelIndex]) {
                //if (depth == depthBuffer[pixelIndex]) {
                    depthBuffer[pixelIndex] = depth;

					//Vector3f normal = bary.x() * rasterized.normals[0] + bary.y() * rasterized.normals[1] + bary.z() * rasterized.normals[2];
                    FragmentInput input = barycentricInterpolation(fi, bary);
                    //if (xCoord < 0 || xCoord >= imageWidth ||
                        //yCoord < 0 || yCoord >= imageHeight
					//)
                        //continue;
                    //Vector3f color = fs->process(input);
                    //Vector3f color = fragmentShader(cb, m.matData, input);
                    ++counter;
                    Vector3f color = m.fragmentShader(cb, m.shaderProps, input);
                    color.x() = std::min(color.x(), 1.0f);
                    color.y() = std::min(color.y(), 1.0f);
                    color.z() = std::min(color.z(), 1.0f);
                    //float invMaxColor = 1.0f / color.maxCoeff();
                    //color *= std::min(1.0f, invMaxColor);

                    backBuffer[startIndex + 0] = color.x() * 255.99;
                    backBuffer[startIndex + 1] = color.y() * 255.99;
                    backBuffer[startIndex + 2] = color.z() * 255.99;
                    backBuffer[startIndex + 3] = 255.99;
                }
            }
        }
    }
    //stop = clock();
    //qDebug() << "rasterize a triangle took " << double(stop - start) / CLOCKS_PER_SEC << " s";
    //qDebug() << "run fragment shader " << counter << " times";
    return counter;
}

int SoftRasterizer::drawTriangleDepth(float* buffer, Triangle& tri)
{
    Triangle screenSpaceTri;

    Vector3f a = tri.v[0].head(3);
    Vector3f b = tri.v[1].head(3);
    Vector3f c = tri.v[2].head(3);

    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
    if (crossNormal.dot(aveNormal) < 0) {
        //qDebug() << "not counter clockwise";
        std::swap(tri.v[0], tri.v[2]);
        std::swap(tri.normals[0], tri.normals[2]);
        std::swap(tri.texCoords[0], tri.texCoords[2]);
        std::swap(tri.colors[0], tri.colors[2]);
    }

    FragmentInput fi[3];
    for (int i = 0; i < 3; ++i) {
        VertexInput vi;
        vi.positionOS = tri.v[i];
        vi.normalOS = tri.normals[i];
        fi[i] = m.vertexShader(cb, m.shaderProps, vi);
    }

    screenSpaceTri.v[0] = screenMapping(imageWidth, imageHeight, fi[0].positionCS);
    screenSpaceTri.normals[0] = fi[0].normalWS;

    screenSpaceTri.v[1] = screenMapping(imageWidth, imageHeight, fi[1].positionCS);
    screenSpaceTri.normals[1] = fi[1].normalWS;

    screenSpaceTri.v[2] = screenMapping(imageWidth, imageHeight, fi[2].positionCS);
    screenSpaceTri.normals[2] = fi[2].normalWS;

    bool clip = true;
    for (int i = 0; i < 3; ++i) {
        if (screenSpaceTri.v[i].x() >= 0 && screenSpaceTri.v[i].x() <= imageWidth &&
            screenSpaceTri.v[i].y() >= 0 && screenSpaceTri.v[i].y() <= imageHeight &&
            screenSpaceTri.v[i].z() >= 0 && screenSpaceTri.v[i].z() < 1
            ) {
            clip = false;
            break;
        }
    }

    if (clip) return 0;


    screenSpaceTri.updateInfo();

    float minx = imageWidth, miny = imageHeight;
    float maxx = 0, maxy = 0;
    for (int i = 0; i < 3; i++) {
        minx = screenSpaceTri.v[i].x() < minx ? screenSpaceTri.v[i].x() : minx;
        maxx = screenSpaceTri.v[i].x() > maxx ? screenSpaceTri.v[i].x() : maxx;

        miny = screenSpaceTri.v[i].y() < miny ? screenSpaceTri.v[i].y() : miny;
        maxy = screenSpaceTri.v[i].y() > maxy ? screenSpaceTri.v[i].y() : maxy;
    }


    int iminx = floor(minx);
    int imaxx = ceil(maxx);
    int iminy = floor(miny);
    int imaxy = ceil(maxy);

    iminx = std::max(0, iminx);
    imaxx = std::min(imageWidth - 1, imaxx);
    iminy = std::max(0, iminy);
    imaxy = std::min(imageHeight - 1, imaxy);

    Vector3f bary(0, 0, 0);

    Vector3f equationResult = screenSpaceTri.precomputeLineEquations(iminx + 0.5, iminy + 0.5);
    for (int x = iminx; x <= imaxx; x++) {
        for (int y = iminy; y <= imaxy; y++) {
            int pixelIndex = x + (imageHeight - 1 - y) * imageWidth;
            //int startIndex = pixelIndex * 4;
            int startIndex = pixelIndex;
            Vector3f currResult;
            int xoffset = x - iminx;
            int yoffset = y - iminy;
            currResult.x() = equationResult.x() + xoffset * screenSpaceTri.edgeCoeffs[0].x() + yoffset * screenSpaceTri.edgeCoeffs[0].y();
            currResult.y() = equationResult.y() + xoffset * screenSpaceTri.edgeCoeffs[1].x() + yoffset * screenSpaceTri.edgeCoeffs[1].y();
            currResult.z() = equationResult.z() + xoffset * screenSpaceTri.edgeCoeffs[2].x() + yoffset * screenSpaceTri.edgeCoeffs[2].y();
            if (screenSpaceTri.pointInsideTriangleFast(x + 0.5, y + 0.5, currResult, bary)) {
                float depth = bary.x() * screenSpaceTri.v[0].z() + bary.y() * screenSpaceTri.v[1].z() + bary.z() * screenSpaceTri.v[2].z();
                //float xCoord = bary.x() * screenSpaceTri.v[0].x() + bary.y() * screenSpaceTri.v[1].x() + bary.z() * screenSpaceTri.v[2].x();
                //float yCoord = bary.x() * screenSpaceTri.v[0].y() + bary.y() * screenSpaceTri.v[1].y() + bary.z() * screenSpaceTri.v[2].y();

                if (depth > 0 && depth < 1 && depth > depthBuffer[pixelIndex]) {
                    depthBuffer[pixelIndex] = depth;
                }
            }
        }
    }
    return 0;
}

int SoftRasterizer::drawTriangleShadow(float* buffer, Triangle& tri)
{
    Triangle screenSpaceTri;

    Vector3f a = tri.v[0].head(3);
    Vector3f b = tri.v[1].head(3);
    Vector3f c = tri.v[2].head(3);

    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
    if (crossNormal.dot(aveNormal) < 0) {
        //qDebug() << "not counter clockwise";
        std::swap(tri.v[0], tri.v[2]);
        std::swap(tri.normals[0], tri.normals[2]);
        std::swap(tri.texCoords[0], tri.texCoords[2]);
        std::swap(tri.colors[0], tri.colors[2]);
    }

    FragmentInput fi[3];
    for (int i = 0; i < 3; ++i) {
        VertexInput vi;
        vi.positionOS = tri.v[i];
        vi.normalOS = tri.normals[i];
        fi[i] = shadowMaterial.vertexShader(shadowCb, shadowMaterial.shaderProps, vi);
        //fi[i] = shadowMaterial.vertexShader(cb, shadowMaterial.shaderProps, vi);
    }

    screenSpaceTri.v[0] = screenMapping(shadowWidth, shadowWidth, fi[0].positionCS);
    screenSpaceTri.normals[0] = fi[0].normalWS;

    screenSpaceTri.v[1] = screenMapping(shadowWidth, shadowWidth, fi[1].positionCS);
    screenSpaceTri.normals[1] = fi[1].normalWS;

    screenSpaceTri.v[2] = screenMapping(shadowWidth, shadowWidth, fi[2].positionCS);
    screenSpaceTri.normals[2] = fi[2].normalWS;

    bool clip = true;
    for (int i = 0; i < 3; ++i) {
        if (screenSpaceTri.v[i].x() >= 0 && screenSpaceTri.v[i].x() <= shadowWidth &&
            screenSpaceTri.v[i].y() >= 0 && screenSpaceTri.v[i].y() <= shadowWidth &&
            screenSpaceTri.v[i].z() >= 0 && screenSpaceTri.v[i].z() < 1
            ) {
            clip = false;
            break;
        }
    }

    if (clip) return 0;


    screenSpaceTri.updateInfo();

    float minx = shadowWidth, miny = shadowWidth;
    float maxx = 0, maxy = 0;
    for (int i = 0; i < 3; i++) {
        minx = screenSpaceTri.v[i].x() < minx ? screenSpaceTri.v[i].x() : minx;
        maxx = screenSpaceTri.v[i].x() > maxx ? screenSpaceTri.v[i].x() : maxx;

        miny = screenSpaceTri.v[i].y() < miny ? screenSpaceTri.v[i].y() : miny;
        maxy = screenSpaceTri.v[i].y() > maxy ? screenSpaceTri.v[i].y() : maxy;
    }


    int iminx = floor(minx);
    int imaxx = ceil(maxx);
    int iminy = floor(miny);
    int imaxy = ceil(maxy);

    iminx = std::max(0, iminx);
    imaxx = std::min(shadowWidth - 1, imaxx);
    iminy = std::max(0, iminy);
    imaxy = std::min(shadowWidth - 1, imaxy);

    Vector3f bary(0, 0, 0);

    Vector3f equationResult = screenSpaceTri.precomputeLineEquations(iminx + 0.5, iminy + 0.5);
    for (int x = iminx; x <= imaxx; x++) {
        for (int y = iminy; y <= imaxy; y++) {
            int pixelIndex = x + (shadowWidth - 1 - y) * shadowWidth;
            //int startIndex = pixelIndex * 4;
            int startIndex = pixelIndex;
            Vector3f currResult;
            int xoffset = x - iminx;
            int yoffset = y - iminy;
            currResult.x() = equationResult.x() + xoffset * screenSpaceTri.edgeCoeffs[0].x() + yoffset * screenSpaceTri.edgeCoeffs[0].y();
            currResult.y() = equationResult.y() + xoffset * screenSpaceTri.edgeCoeffs[1].x() + yoffset * screenSpaceTri.edgeCoeffs[1].y();
            currResult.z() = equationResult.z() + xoffset * screenSpaceTri.edgeCoeffs[2].x() + yoffset * screenSpaceTri.edgeCoeffs[2].y();
            if (screenSpaceTri.pointInsideTriangleFast(x + 0.5, y + 0.5, currResult, bary)) {
                float depth = bary.x() * screenSpaceTri.v[0].z() + bary.y() * screenSpaceTri.v[1].z() + bary.z() * screenSpaceTri.v[2].z();
                //float xCoord = bary.x() * screenSpaceTri.v[0].x() + bary.y() * screenSpaceTri.v[1].x() + bary.z() * screenSpaceTri.v[2].x();
                //float yCoord = bary.x() * screenSpaceTri.v[0].y() + bary.y() * screenSpaceTri.v[1].y() + bary.z() * screenSpaceTri.v[2].y();

                if (depth > 0 && depth < 1 && depth > shadowMaps[0][pixelIndex]) {
                    shadowMaps[0][pixelIndex] = depth;
                }
            }
        }
    }
    return 0;
}

void SoftRasterizer::setClearColor(const Vector4f& color)
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

void SoftRasterizer::clear()
{
    //memset(backBuffer, 255, bufferSize);
    //memcpy(backBuffer, clearBuffer.data(), bufferSize);
    std::fill(depthBuffer.begin(), depthBuffer.end(), 0);

    std::fill(shadowMaps[0].begin(), shadowMaps[0].end(), 0.0f);
    std::fill(shadowMaps[1].begin(), shadowMaps[1].end(), 0.0f);
    std::fill(shadowMaps[2].begin(), shadowMaps[2].end(), 0.0f);
    std::fill(shadowMaps[3].begin(), shadowMaps[3].end(), 0.0f);
}

Vector4f SoftRasterizer::screenMapping(int screenWidth, int screenHeight, Vector4f clipSpaceCoord) const
{
    float invW = 1.0 / clipSpaceCoord.w();
    Vector4f NDC(
        clipSpaceCoord.x() * invW,
        clipSpaceCoord.y() * invW,
        clipSpaceCoord.z() * invW,
        invW
    );
    Vector4f screenSpaceCoord(
        (NDC.x()+1) * screenWidth * 0.5,
        (NDC.y()+1) * screenHeight * 0.5,
        NDC.z(),
        NDC.w()
    );
    return screenSpaceCoord;
}

void SoftRasterizer::loadModel(const std::string& filename)
{
    std::string inputfile = "./models/spot/spot_triangulated_good.obj";
    //std::string inputfile = "./models/bunny/bunny.obj";
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./models/spot/"; // Path to material files
    //reader_config.mtl_search_path = "./models/bunny/"; // Path to material files

    //tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(inputfile, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }
}

void SoftRasterizer::paintEvent(QPaintEvent*)
{
    //MyTransform t;
    //t.rotate(Vector3f(0, 1, 0), M_PI/4);
    //t.translate(Vector3f(100, 200, 123));
    //t.print();
    clock_t currTime = clock();
    deltaTime = float(currTime - lastFrameTime) / CLOCKS_PER_SEC;
    lastFrameTime = currTime;

    updateConstantBuffer();
    updateShadowConstantBuffer();

    qDebug("shadowCam.pos = (%f, %f, %f)", shadowCam.pos.x(), shadowCam.pos.y(), shadowCam.pos.z());
    qDebug("shadowCam.fov = (%f)", shadowCam.vfov);
    //Matrix4f worldToCamera = cam.getWorldToCamera();
    //Matrix4f projection = cam.getProjection();
    //M = Matrix4f::Identity();
    //V = worldToCamera;
    //P = projection;
    //MVP = projection * worldToCamera;

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();


    clock_t start, end;

	clear();

    //uchar* backBuffer = new uchar[bufferSize];
    //memset(backBuffer, 0, bufferSize);

    start = clock();
    //for (int k = 0; k < 1000; ++k) {

    // On CPU and simple scene, earlyZ may slow down render speed
    if (earlyZ)
		renderDepthMap();

    renderShadowMap();

    //render();

	end = clock();
	label.setPixmap(QPixmap::fromImage(img));
    qDebug() << CLOCKS_PER_SEC / double(end - start) << " FPS";
    qDebug() << double(end - start) / CLOCKS_PER_SEC  << " s";
}

void SoftRasterizer::keyPressEvent(QKeyEvent* e)
{
    const float speed = 0.25;
    
    float step = speed;

    Vector3f offset = Vector3f::Zero();
    Vector3f front = cam.z;
    Vector3f right = cam.x;
    Vector3f up = cam.y;
    switch (e->key())
    {
    case Qt::Key_W:
        offset = front;
        break;
    case Qt::Key_S:
        offset =  -front;
        break;
    case Qt::Key_A:
        offset =  -right;
        break;
    case Qt::Key_D:
        offset =  right;
        break;
    case Qt::Key_Q:
        offset =  -up;
        break;
    case Qt::Key_E:
        offset =  up;
        break;
    case Qt::Key_Space:
		resetCamera();
		resetShadowCamera();
		break;
    }
    offset *= step;
    cam.setPosition(cam.pos + offset);
    cam.setTarget(cam.target + offset);
	cam.updateTransform();

    shadowCam.setPosition(shadowCam.pos + offset);
    shadowCam.setTarget(shadowCam.target + offset);
    shadowCam.updateTransform();

    assert(cam.pos == shadowCam.pos);
}

void SoftRasterizer::mousePressEvent(QMouseEvent* e)
{
    lastFrameMousePos = e->globalPos();
}

void SoftRasterizer::mouseMoveEvent(QMouseEvent* e)
{
    QPoint currFrameMousePos = e->globalPos();
    QPoint offset = (currFrameMousePos - lastFrameMousePos);

    Vector2f rotation(float(offset.x()) / float(imageWidth), float(offset.y()) / float(imageHeight));
    float rotationRadian = rotation.norm() * M_PI;
    Vector2f rotationRadians = rotation * M_PI;

    Vector3f rotationAxis(offset.y(), offset.x(), 0);

    cam.rotate(rotationAxis, rotationRadian);
    //cam.rotateAroundY(rotationRadians.x());
    //cam.rotateAroundX(rotationRadians.y());
    cam.updateTransform();

    shadowCam.rotate(rotationAxis, rotationRadian);
    shadowCam.updateTransform();

    lastFrameMousePos = currFrameMousePos;
}

void SoftRasterizer::wheelEvent(QWheelEvent* e)
{
    const float speed = 0.01;
    float step = speed * e->angleDelta().y();
    float fov = cam.vfov;
    fov -= step;
    fov = std::max(fov, 0.0f);
    fov = std::min(fov, 180.0f);
    qDebug() << "fov = " << fov;
    cam.setVFov(fov);
    cam.updateProjection();

    shadowCam.setVFov(fov);
    shadowCam.updateProjection();
}

void SoftRasterizer::testEigen()
{
    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    qDebug() << m(0, 0) << m(1, 0) << m(0, 1) << m(1, 1);
}

void SoftRasterizer::resetCamera()
{
    cam.setPosition(Vector3f(0, 0, -1.0));
    cam.setTarget(Vector3f(0, 0, 0));
    cam.setUp(Vector3f(0, 1, 0));

    cam.setVFov(90);
    cam.setRatio(float(imageWidth) / float(imageHeight));
    cam.setNear(0.1);
    cam.setFar(50);
}

void SoftRasterizer::resetShadowCamera()
{
    //shadowCam.setPosition(mainLight.position);
    //shadowCam.setTarget(mainLight.position - mainLight.direction);
    //shadowCam.setUp(Vector3f(0, 0, 1));

    //shadowCam.setVFov(90);
    //shadowCam.setRatio(1);
    //shadowCam.setNear(0.1);
    //shadowCam.setFar(50);

    shadowCam.setPosition(Vector3f(0, 0, -1.0));
    shadowCam.setTarget(Vector3f(0, 0, 0));
    shadowCam.setUp(Vector3f(0, 1, 0));

    shadowCam.setVFov(90);
    shadowCam.setRatio(1);
    shadowCam.setNear(0.1);
    shadowCam.setFar(50);
}

void SoftRasterizer::updateConstantBuffer()
{
    cb.cameraPosition = cam.pos;
    cb.objectToWorld = modelMatrix;
    cb.worldToCamera = cam.getWorldToCamera();
    cb.projection = cam.getProjection();
}

void SoftRasterizer::updateShadowConstantBuffer()
{
    //shadowCb.cameraPosition = mainLight.position;
    //shadowCb.objectToWorld = modelMatrix;
    //shadowCb.worldToCamera = shadowCam.getWorldToCamera();
    //shadowCb.projection = shadowCam.getProjection();
    shadowCb.cameraPosition = shadowCam.pos;
    shadowCb.objectToWorld = modelMatrix;
    shadowCb.worldToCamera = shadowCam.getWorldToCamera();
    shadowCb.projection = shadowCam.getProjection();
}

FragmentInput SoftRasterizer::barycentricInterpolation(const FragmentInput* vertices, Vector3f& baryCoord)
{
    FragmentInput output;
    output.positionWS = baryCoord.x() * vertices[0].positionWS + baryCoord.y() * vertices[1].positionWS + baryCoord.z() * vertices[2].positionWS;
    output.positionCS = baryCoord.x() * vertices[0].positionCS + baryCoord.y() * vertices[1].positionCS + baryCoord.z() * vertices[2].positionCS;
    output.texCoord = baryCoord.x() * vertices[0].texCoord + baryCoord.y() * vertices[1].texCoord + baryCoord.z() * vertices[2].texCoord;
    output.shadowCoord = baryCoord.x() * vertices[0].shadowCoord + baryCoord.y() * vertices[1].shadowCoord + baryCoord.z() * vertices[2].shadowCoord;
    output.normalWS = baryCoord.x() * vertices[0].normalWS + baryCoord.y() * vertices[1].normalWS + baryCoord.z() * vertices[2].normalWS;
    output.normalWS.normalize();
    return output;
}

void SoftRasterizer::renderDepthMap()
{
    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    int counter = 0;
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            Triangle tri;

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                tri.v[v] = Vector4f(vx, vy, vz, 1.0);
            }
            index_offset += fv;
            counter += drawTriangleDepth(depthBuffer.data(), tri);
        }
    }
}

void SoftRasterizer::renderShadowMap()
{
    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    int counter = 0;
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            Triangle tri;

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                tri.v[v] = Vector4f(vx, vy, vz, 1.0);
            }
            index_offset += fv;
            counter += drawTriangleShadow(shadowMaps[0].data(), tri);
        }
    }
    std::vector<uchar> temp(bufferSize);
    //uchar* temp = new uchar[bufferSize];
    for (int i = 0; i < shadowWidth; ++i) {
        for (int j = 0; j < shadowWidth; ++j) {
            int index = i + j * imageWidth;
            int pixelIndex = i + j * shadowWidth;
			temp[4 * index] = uchar(shadowMaps[0][pixelIndex] * 255.99f);
            //qDebug("shadowMaps[0] = %f", shadows
			temp[4 * index + 3] = 255;
        }
    }
    memcpy(frontBuffer, temp.data(), bufferSize);
    //memcpy(frontBuffer, temp, bufferSize);
    //for (int i = 0; i < 1000; ++i) frontBuffer[i * 4] = 255;
}

void SoftRasterizer::render()
{
    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    uchar* backBuffer = new uchar[bufferSize];
    memset(backBuffer, 0, bufferSize);

    int counter = 0;
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            Triangle tri;

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                tri.v[v] = Vector4f(vx, vy, vz, 1.0);
                //qDebug() << vx << ", " << vy << ", " << vz;

                // Check if `normal_index` is zero or positive. negative = no normal data
                if (idx.normal_index >= 0) {
                    tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
                    tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
                    tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];

                    tri.normals[v] = Vector3f(nx, ny, nz).normalized();
                }

                // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                if (idx.texcoord_index >= 0) {
                    tinyobj::real_t tx = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
                    tinyobj::real_t ty = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];

                    //qDebug() << tx << ", " << ty;
                    tri.texCoords[v] = Vector2f(tx, ty);
                }

                // Optional: vertex colors
                tinyobj::real_t red = attrib.colors[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t green = attrib.colors[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t blue = attrib.colors[3 * size_t(idx.vertex_index) + 2];

                //qDebug() << red << ", " << green << ", " << blue;
                tri.colors[v] = Vector4f(red, green, blue, 255);
            }
            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];

            counter += drawTriangle(backBuffer, tri);
        }
    }
    qDebug() << "run fragment shader " << counter << " times";
    memcpy(frontBuffer, backBuffer, bufferSize);
    delete[] backBuffer;
}

