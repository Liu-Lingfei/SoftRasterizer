#include "SoftRasterizer.h"

SoftRasterizer::SoftRasterizer(QWidget* parent)
    :
    QMainWindow(parent),
    img(imageWidth, imageHeight, QImage::Format_RGBA8888),
    frontBuffer(img.bits()),
    lastFrameMousePos(0, 0),
    deltaTime(0)
{
    ui.setupUi(this);
    resize(1920, 1080);

    //r.reset(new Renderer(imageWidth, imageHeight, shadowWidth));
    //r = new FastRenderer(imageWidth, imageHeight, shadowWidth);
    //r->bindReader(&reader);
    //r->setCamera(&cam);
    //r->setShadowCamera(&shadowCam);
    //r->setMainLight(&mainLight);
    //r->setMaterial(&m);
    //r->setShadowMaterial(&shadowMaterial);
    //r->setModelMatrix(&modelMatrix);
    //r->bindConstantBuffer(&cb);
    //r->setShadowConstantBuffer(&shadowCb);


    img.fill(Qt::white);
    label.setParent(this);
    label.setGeometry(0, 0, img.width(), img.height());
    label.show();

    //clearBuffer = new uchar[bufferSize];
    //setClearColor(Vector4f(0, 0, 0, 255));

    //cam.setVFov(90);
    //cam.setRatio(float(imageWidth) / float(imageHeight));
    //cam.setNear(1);
    //cam.setFar(50);
    //resetCamera();
    //cam.updateTransform();
    //cam.updateProjection();

    //loadModel("");

    //mainLight.color = Vector3f(1, 1, 1);
    ////mainLight.direction = Vector3f(0, 1, 0);
    ////mainLight.position = Vector3f(0, 10, 0);
    //mainLight.direction = Vector3f(1, 1, 0);
    //mainLight.position = Vector3f(10, 10, 0);

    //cb.mainLight = mainLight;

    //m.shaderProps.albedo = Vector3f(0.9, 0.5, 0.5);
    //m.shaderProps.specular = Vector3f(0.1, 0.1, 0.1);
    //m.shaderProps.ambientColor = Vector3f(0.1, 0.1, 0.1);

    //m.shaderProps.smoothness = 0.8;
    //m.shaderProps.metallic = 0.8;

    //m.vertexShader = CommonVertexShader;
    //m.fragmentShader = UnityPBRFragmentShader;

    //shadowMaterial.vertexShader = ShadowMapVertexShader;
    //shadowMaterial.fragmentShader = ShadowMapFragmentShader;

    //resetShadowCamera();
    //shadowCam.updateProjection();
    //shadowCam.updateTransform();
    //updateShadowConstantBuffer();

    //updateConstantBuffer();
}

SoftRasterizer::~SoftRasterizer()
{
}

//int SoftRasterizer::drawTriangle(uchar* backBuffer, Triangle& tri)
//{
//    Triangle screenSpaceTri;
//
//    Vector3f a = tri.v[0].head(3);
//    Vector3f b = tri.v[1].head(3);
//    Vector3f c = tri.v[2].head(3);
//
//    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
//    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
//    Vector3f avePos = (a + b + c)/3;
//
//
//    if (crossNormal.dot(aveNormal) < 0) {
//        //qDebug() << "not counter clockwise";
//        std::swap(tri.v[0], tri.v[2]);
//        std::swap(tri.normals[0], tri.normals[2]);
//        std::swap(tri.texCoords[0], tri.texCoords[2]);
//        std::swap(tri.colors[0], tri.colors[2]);
//    }
//
//
//    FragmentInput fi[3];
//    for (int i = 0; i < 3; ++i) {
//		VertexInput vi;
//		vi.positionOS = tri.v[i];
//        vi.normalOS = tri.normals[i];
//        fi[i] = m.vertexShader(cb, m.shaderProps, vi);
//    }
//
//    Vector3f viewDir = cam.pos - avePos;
//    aveNormal = (fi[0].normalWS + fi[1].normalWS + fi[2].normalWS).normalized();
//    if (viewDir.dot(aveNormal) < 0) {
//        return 0;
//    }
//
//    if (fi[0].positionCS.w() < 0 || fi[1].positionCS.w() < 0 || fi[2].positionCS.w() < 0)
//        return 0;
//
//    screenSpaceTri.v[0] = screenMapping(imageWidth, imageHeight, fi[0].positionCS);
//    screenSpaceTri.normals[0] = fi[0].normalWS;
//
//    screenSpaceTri.v[1] = screenMapping(imageWidth, imageHeight, fi[1].positionCS);
//    screenSpaceTri.normals[1] = fi[1].normalWS;
//
//    screenSpaceTri.v[2] = screenMapping(imageWidth, imageHeight, fi[2].positionCS);
//    screenSpaceTri.normals[2] = fi[2].normalWS;
//
//    screenSpaceTri.updateInfo();
//    //rasterized.updateInfo();
//
//    float minx = imageWidth, miny = imageHeight;
//    float maxx = 0, maxy = 0;
//    for (int i = 0; i < 3; i++) {
//        minx = screenSpaceTri.v[i].x() < minx ? screenSpaceTri.v[i].x() : minx;
//        maxx = screenSpaceTri.v[i].x() > maxx ? screenSpaceTri.v[i].x() : maxx;
//
//        miny = screenSpaceTri.v[i].y() < miny ? screenSpaceTri.v[i].y() : miny;
//        maxy = screenSpaceTri.v[i].y() > maxy ? screenSpaceTri.v[i].y() : maxy;
//    }
//
//
//    int iminx = floor(minx);
//    int imaxx = ceil(maxx);
//    int iminy = floor(miny);
//    int imaxy = ceil(maxy);
//
//    iminx = std::max(0, iminx);
//    imaxx = std::min(imageWidth - 1, imaxx);
//    iminy = std::max(0, iminy);
//    imaxy = std::min(imageHeight - 1, imaxy);
//
//    //qDebug("iminx = %d, imaxx = %d, iminy = %d, imaxy = %d", iminx, imaxx, iminy, imaxy);
//
//    //clock_t start, stop;
//    //start = clock();
//    int counter = 0;
//    Vector3f baryCoord(0, 0, 0);
//    Vector3f perspectiveBaryCoord(0, 0, 0);
//
//
//	for (int y = iminy; y <= imaxy; y++) {
//		//Vector3f equationResult = screenSpaceTri.precomputeLineEquations(iminx + 0.5, y + 0.5);
//		for (int x = iminx; x <= imaxx; x++) {
//            int pixelIndex = x + (imageHeight - 1 - y) * imageWidth;
//            int startIndex = pixelIndex*4;
//            //Vector3f currResult;
//            //int xoffset = x - iminx;
//            //int yoffset = y - iminy;
//            //currResult.x() = equationResult.x() + xoffset * screenSpaceTri.edgeCoeffs[0].x() + yoffset * screenSpaceTri.edgeCoeffs[0].y();
//            //currResult.y() = equationResult.y() + xoffset * screenSpaceTri.edgeCoeffs[1].x() + yoffset * screenSpaceTri.edgeCoeffs[1].y();
//            //currResult.z() = equationResult.z() + xoffset * screenSpaceTri.edgeCoeffs[2].x() + yoffset * screenSpaceTri.edgeCoeffs[2].y();
//            //if (screenSpaceTri.pointInsideTriangleFast(x+0.5, y+0.5, currResult, bary)) {
//            if (screenSpaceTri.pointInsideTriangle(x+0.5, y+0.5, baryCoord, perspectiveBaryCoord)) {
//                //float realZ =
//                    //perspectiveBaryCoord.x() * (1 / screenSpaceTri.v[0].w()) +
//                    //perspectiveBaryCoord.y() * (1 / screenSpaceTri.v[1].w()) +
//                    //perspectiveBaryCoord.z() * (1 / screenSpaceTri.v[2].w());
//                //float realDepth = (cam.far / realZ - 1) / (cam.far / cam.near - 1);
//                float depth = baryCoord.x() * screenSpaceTri.v[0].z() + baryCoord.y() * screenSpaceTri.v[1].z() + baryCoord.z() * screenSpaceTri.v[2].z();
//                //float xCoord = baryCoord.x() * screenSpaceTri.v[0].x() + baryCoord.y() * screenSpaceTri.v[1].x() + baryCoord.z() * screenSpaceTri.v[2].x();
//                //float yCoord = baryCoord.x() * screenSpaceTri.v[0].y() + baryCoord.y() * screenSpaceTri.v[1].y() + baryCoord.z() * screenSpaceTri.v[2].y();
//
//                //if (
//                //    x == 620 && imageHeight - 1 - y == 617
//                //    ) {
//                //    qDebug("BaryCoord = (%f, %f, %f)", baryCoord.x(), baryCoord.y(), baryCoord.z());
//                //    qDebug("perspectiveBaryCoord = (%f, %f, %f)", perspectiveBaryCoord.x(), perspectiveBaryCoord.y(), perspectiveBaryCoord.z());
//                //    qDebug("tri.v[0] = (%f, %f, %f, %f)", tri.v[0].x(), tri.v[0].y(), tri.v[0].z(), tri.v[0].w());
//                //    qDebug("tri.v[1] = (%f, %f, %f, %f)", tri.v[1].x(), tri.v[1].y(), tri.v[1].z(), tri.v[1].w());
//                //    qDebug("tri.v[2] = (%f, %f, %f, %f)", tri.v[2].x(), tri.v[2].y(), tri.v[2].z(), tri.v[2].w());
//                //    qDebug("screenSpaceTri.v[0] = (%f, %f, %f, %f)", screenSpaceTri.v[0].x(), screenSpaceTri.v[0].y(), screenSpaceTri.v[0].z(), screenSpaceTri.v[0].w());
//                //    qDebug("screenSpaceTri.v[1] = (%f, %f, %f, %f)", screenSpaceTri.v[1].x(), screenSpaceTri.v[1].y(), screenSpaceTri.v[1].z(), screenSpaceTri.v[1].w());
//                //    qDebug("screenSpaceTri.v[2] = (%f, %f, %f, %f)", screenSpaceTri.v[2].x(), screenSpaceTri.v[2].y(), screenSpaceTri.v[2].z(), screenSpaceTri.v[2].w());
//
//                //    qDebug("xCoord = %f, yCoord = %f, x = %d, y = %d", xCoord, yCoord, x, y);
//                //    qDebug("input.positionCS = (%f, %f, %f, %f)", input.positionCS.x(), input.positionCS.y(), input.positionCS.z(), input.positionCS.w());
//                //    qDebug("input.positionWS = (%f, %f, %f, %f)", input.positionWS.x(), input.positionWS.y(), input.positionWS.z(), input.positionWS.w());
//                //    qDebug("realZ = %f", realZ);
//                //    qDebug("realDepth = %f, depth = %f", realDepth, depth);
//                //    qDebug("depthBuffer = %f", depthBuffer[pixelIndex]);
//                //    qDebug("");
//                //    continue;
//                //}
//
//
//                //if (realDepth > 0 && realDepth < 1 && realDepth > depthBuffer[pixelIndex])
//                if (depth > 0 && depth < 1 && depth > depthBuffer[pixelIndex])
//                {
//
//                    depthBuffer[pixelIndex] = depth;
//					FragmentInput input = barycentricInterpolation(fi, baryCoord, perspectiveBaryCoord);
//                    //qDebug("depth = %f, input depth = %f", depth, input.positionCS.z() / input.positionCS.w());
//                    //Vector3f color = fs->process(input);
//                    //Vector3f color = fragmentShader(cb, m.matData, input);
//                    ++counter;
//                    Vector3f color = m.fragmentShader(cb, m.shaderProps, input);
//                    color.x() = std::min(color.x(), 1.0f);
//                    color.y() = std::min(color.y(), 1.0f);
//                    color.z() = std::min(color.z(), 1.0f);
//
//                    backBuffer[startIndex + 0] = color.x() * 255.99;
//                    backBuffer[startIndex + 1] = color.y() * 255.99;
//                    backBuffer[startIndex + 2] = color.z() * 255.99;
//                    backBuffer[startIndex + 3] = 255.99;
//                }
//            }
//        }
//    }
//    return counter;
//}
//
//int SoftRasterizer::drawTriangleDepth(float* buffer, Triangle& tri)
//{
//    Triangle screenSpaceTri;
//
//    Vector3f a = tri.v[0].head(3);
//    Vector3f b = tri.v[1].head(3);
//    Vector3f c = tri.v[2].head(3);
//
//    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
//    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
//    Vector3f avePos = (a + b + c)/3;
//
//    if (crossNormal.dot(aveNormal) < 0) {
//        //qDebug() << "not counter clockwise";
//        std::swap(tri.v[0], tri.v[2]);
//        std::swap(tri.normals[0], tri.normals[2]);
//        std::swap(tri.texCoords[0], tri.texCoords[2]);
//        std::swap(tri.colors[0], tri.colors[2]);
//    }
//
//
//    FragmentInput fi[3];
//    for (int i = 0; i < 3; ++i) {
//        VertexInput vi;
//        vi.positionOS = tri.v[i];
//        vi.normalOS = tri.normals[i];
//        fi[i] = m.vertexShader(cb, m.shaderProps, vi);
//    }
//
//    Vector3f viewDir = cam.pos - avePos;
//    aveNormal = (fi[0].normalWS + fi[1].normalWS + fi[2].normalWS).normalized();
//    if (viewDir.dot(aveNormal) < 0) {
//        return 0;
//    }
//
//    if (fi[0].positionCS.w() < 0 || fi[1].positionCS.w() < 0 || fi[2].positionCS.w() < 0)
//        return 0;
//
//    screenSpaceTri.v[0] = screenMapping(imageWidth, imageHeight, fi[0].positionCS);
//    screenSpaceTri.normals[0] = fi[0].normalWS;
//
//    screenSpaceTri.v[1] = screenMapping(imageWidth, imageHeight, fi[1].positionCS);
//    screenSpaceTri.normals[1] = fi[1].normalWS;
//
//    screenSpaceTri.v[2] = screenMapping(imageWidth, imageHeight, fi[2].positionCS);
//    screenSpaceTri.normals[2] = fi[2].normalWS;
//
//    screenSpaceTri.updateInfo();
//
//    float minx = imageWidth, miny = imageHeight;
//    float maxx = 0, maxy = 0;
//    for (int i = 0; i < 3; i++) {
//        minx = screenSpaceTri.v[i].x() < minx ? screenSpaceTri.v[i].x() : minx;
//        maxx = screenSpaceTri.v[i].x() > maxx ? screenSpaceTri.v[i].x() : maxx;
//
//        miny = screenSpaceTri.v[i].y() < miny ? screenSpaceTri.v[i].y() : miny;
//        maxy = screenSpaceTri.v[i].y() > maxy ? screenSpaceTri.v[i].y() : maxy;
//    }
//
//
//    int iminx = floor(minx);
//    int imaxx = ceil(maxx);
//    int iminy = floor(miny);
//    int imaxy = ceil(maxy);
//
//    iminx = std::max(0, iminx);
//    imaxx = std::min(imageWidth - 1, imaxx);
//    iminy = std::max(0, iminy);
//    imaxy = std::min(imageHeight - 1, imaxy);
//
//    Vector3f baryCoord(0, 0, 0);
//    Vector3f perspectiveBaryCoord(0, 0, 0);
//
//    //Vector3f equationResult = screenSpaceTri.precomputeLineEquations(iminx + 0.5, iminy + 0.5);
//    for (int x = iminx; x <= imaxx; x++) {
//        for (int y = iminy; y <= imaxy; y++) {
//            int pixelIndex = x + (imageHeight - 1 - y) * imageWidth;
//            //int startIndex = pixelIndex * 4;
//            int startIndex = pixelIndex;
//            Vector3f currResult;
//            //int xoffset = x - iminx;
//            //int yoffset = y - iminy;
//            //currResult.x() = equationResult.x() + xoffset * screenSpaceTri.edgeCoeffs[0].x() + yoffset * screenSpaceTri.edgeCoeffs[0].y();
//            //currResult.y() = equationResult.y() + xoffset * screenSpaceTri.edgeCoeffs[1].x() + yoffset * screenSpaceTri.edgeCoeffs[1].y();
//            //currResult.z() = equationResult.z() + xoffset * screenSpaceTri.edgeCoeffs[2].x() + yoffset * screenSpaceTri.edgeCoeffs[2].y();
//            if (screenSpaceTri.pointInsideTriangle(x+0.5, y+0.5, baryCoord, perspectiveBaryCoord)) {
//            //if (screenSpaceTri.pointInsideTriangleFast(x + 0.5, y + 0.5, currResult, bary)) {
//                float depth = baryCoord.x() * screenSpaceTri.v[0].z() + baryCoord.y() * screenSpaceTri.v[1].z() + baryCoord.z() * screenSpaceTri.v[2].z();
//                //float xCoord = baryCoord.x() * screenSpaceTri.v[0].x() + baryCoord.y() * screenSpaceTri.v[1].x() + baryCoord.z() * screenSpaceTri.v[2].x();
//                //float yCoord = baryCoord.x() * screenSpaceTri.v[0].y() + baryCoord.y() * screenSpaceTri.v[1].y() + baryCoord.z() * screenSpaceTri.v[2].y();
//
//                if (depth > 0 && depth < 1 && depth > depthBuffer[pixelIndex]) {
//                    depthBuffer[pixelIndex] = depth;
//                }
//            }
//        }
//    }
//    return 0;
//}

//int SoftRasterizer::drawTriangleShadow(float* buffer, Triangle& tri)
//{
//    Triangle screenSpaceTri;
//
//    Vector3f a = tri.v[0].head(3);
//    Vector3f b = tri.v[1].head(3);
//    Vector3f c = tri.v[2].head(3);
//
//    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
//    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
//    Vector3f avePos = (a + b + c)/3;
//    if (crossNormal.dot(aveNormal) < 0) {
//        //qDebug() << "not counter clockwise";
//        std::swap(tri.v[0], tri.v[2]);
//        std::swap(tri.normals[0], tri.normals[2]);
//        std::swap(tri.texCoords[0], tri.texCoords[2]);
//        std::swap(tri.colors[0], tri.colors[2]);
//    }
//
//    FragmentInput fi[3];
//    for (int i = 0; i < 3; ++i) {
//        VertexInput vi;
//        vi.positionOS = tri.v[i];
//        vi.normalOS = tri.normals[i];
//        fi[i] = shadowMaterial.vertexShader(shadowCb, shadowMaterial.shaderProps, vi);
//        //fi[i] = shadowMaterial.vertexShader(cb, shadowMaterial.shaderProps, vi);
//    }
//
//    Vector3f viewDir = shadowCam.pos - shadowCam.target;
//    aveNormal = (fi[0].normalWS + fi[1].normalWS + fi[2].normalWS).normalized();
//    if (viewDir.dot(aveNormal) < 0) {
//        return 0;
//    }
//
//    if (fi[0].positionCS.w() < 0 || fi[1].positionCS.w() < 0 || fi[2].positionCS.w() < 0)
//        return 0;
//
//    screenSpaceTri.v[0] = screenMapping(shadowWidth, shadowWidth, fi[0].positionCS);
//    screenSpaceTri.normals[0] = fi[0].normalWS;
//
//    screenSpaceTri.v[1] = screenMapping(shadowWidth, shadowWidth, fi[1].positionCS);
//    screenSpaceTri.normals[1] = fi[1].normalWS;
//
//    screenSpaceTri.v[2] = screenMapping(shadowWidth, shadowWidth, fi[2].positionCS);
//    screenSpaceTri.normals[2] = fi[2].normalWS;
//
//    screenSpaceTri.updateInfo();
//
//    float minx = shadowWidth, miny = shadowWidth;
//    float maxx = 0, maxy = 0;
//    for (int i = 0; i < 3; i++) {
//        minx = screenSpaceTri.v[i].x() < minx ? screenSpaceTri.v[i].x() : minx;
//        maxx = screenSpaceTri.v[i].x() > maxx ? screenSpaceTri.v[i].x() : maxx;
//
//        miny = screenSpaceTri.v[i].y() < miny ? screenSpaceTri.v[i].y() : miny;
//        maxy = screenSpaceTri.v[i].y() > maxy ? screenSpaceTri.v[i].y() : maxy;
//    }
//
//
//    int iminx = floor(minx);
//    int imaxx = ceil(maxx);
//    int iminy = floor(miny);
//    int imaxy = ceil(maxy);
//
//    iminx = std::max(0, iminx);
//    imaxx = std::min(shadowWidth - 1, imaxx);
//    iminy = std::max(0, iminy);
//    imaxy = std::min(shadowWidth - 1, imaxy);
//
//    Vector3f baryCoord(0, 0, 0);
//    Vector3f perspectiveBaryCoord(0, 0, 0);
//
//    Vector3f equationResult = screenSpaceTri.precomputeLineEquations(iminx + 0.5, iminy + 0.5);
//    for (int x = iminx; x <= imaxx; x++) {
//        for (int y = iminy; y <= imaxy; y++) {
//            int pixelIndex = x + (shadowWidth - 1 - y) * shadowWidth;
//            //int startIndex = pixelIndex * 4;
//            int startIndex = pixelIndex;
//            Vector3f currResult;
//            int xoffset = x - iminx;
//            int yoffset = y - iminy;
//            currResult.x() = equationResult.x() + xoffset * screenSpaceTri.edgeCoeffs[0].x() + yoffset * screenSpaceTri.edgeCoeffs[0].y();
//            currResult.y() = equationResult.y() + xoffset * screenSpaceTri.edgeCoeffs[1].x() + yoffset * screenSpaceTri.edgeCoeffs[1].y();
//            currResult.z() = equationResult.z() + xoffset * screenSpaceTri.edgeCoeffs[2].x() + yoffset * screenSpaceTri.edgeCoeffs[2].y();
//            //if (screenSpaceTri.pointInsideTriangleFast(x + 0.5, y + 0.5, currResult, bary)) {
//            if (screenSpaceTri.pointInsideTriangle(x+0.5, y+0.5, baryCoord, perspectiveBaryCoord)) {
//                float depth = baryCoord.x() * screenSpaceTri.v[0].z() + baryCoord.y() * screenSpaceTri.v[1].z() + baryCoord.z() * screenSpaceTri.v[2].z();
//                //float xCoord = bary.x() * screenSpaceTri.v[0].x() + bary.y() * screenSpaceTri.v[1].x() + bary.z() * screenSpaceTri.v[2].x();
//                //float yCoord = bary.x() * screenSpaceTri.v[0].y() + bary.y() * screenSpaceTri.v[1].y() + bary.z() * screenSpaceTri.v[2].y();
//
//                if (depth > 0 && depth < 1 && depth > shadowMaps[0][pixelIndex]) {
//                    shadowMaps[0][pixelIndex] = depth;
//                }
//            }
//        }
//    }
//    return 0;
//}

//void SoftRasterizer::setClearColor(const Vector4f& color)
//{
//    int pixIndex = 0;
//    clearColor = color;
//    for (int i = 0; i < imageWidth; ++i) {
//        for (int j = 0; j < imageHeight; ++j) {
//            clearBuffer[pixIndex + 0] = clearColor.x();
//            clearBuffer[pixIndex + 1] = clearColor.y();
//            clearBuffer[pixIndex + 2] = clearColor.z();
//            clearBuffer[pixIndex + 3] = clearColor.w();
//            pixIndex += 4;
//        }
//    }
//}

//void SoftRasterizer::clear()
//{
//    //memset(backBuffer, 255, bufferSize);
//    //memcpy(backBuffer, clearBuffer.data(), bufferSize);
//    std::fill(depthBuffer.begin(), depthBuffer.end(), 0);
//
//    std::fill(shadowMaps[0].begin(), shadowMaps[0].end(), 0.0f);
//    std::fill(shadowMaps[1].begin(), shadowMaps[1].end(), 0.0f);
//    std::fill(shadowMaps[2].begin(), shadowMaps[2].end(), 0.0f);
//    std::fill(shadowMaps[3].begin(), shadowMaps[3].end(), 0.0f);
//}

//Vector4f SoftRasterizer::screenMapping(int screenWidth, int screenHeight, Vector4f clipSpaceCoord) const
//{
//    float invW = 1.0 / clipSpaceCoord.w();
//    Vector4f NDC(
//        clipSpaceCoord.x() * invW,
//        clipSpaceCoord.y() * invW,
//        clipSpaceCoord.z() * invW,
//        invW
//    );
//    Vector4f screenSpaceCoord(
//        (NDC.x()+1) * screenWidth * 0.5,
//        (NDC.y()+1) * screenHeight * 0.5,
//        NDC.z(),
//        NDC.w()
//    );
//    return screenSpaceCoord;
//}

//void SoftRasterizer::loadModel(const std::string& filename)
//{
//    std::string inputfile = "./models/spot/spot_triangulated_good.obj";
//    //std::string inputfile = "./models/bunny/bunny.obj";
//    tinyobj::ObjReaderConfig reader_config;
//    reader_config.mtl_search_path = "./models/spot/"; // Path to material files
//    //reader_config.mtl_search_path = "./models/bunny/"; // Path to material files
//
//    //tinyobj::ObjReader reader;
//
//    if (!reader.ParseFromFile(inputfile, reader_config)) {
//        if (!reader.Error().empty()) {
//            std::cerr << "TinyObjReader: " << reader.Error();
//        }
//        exit(1);
//    }
//
//    if (!reader.Warning().empty()) {
//        std::cout << "TinyObjReader: " << reader.Warning();
//    }
//}

void func(QString name)
{
    //for (int i = 0; i < 10; ++i) {
		qDebug() << 0 << ": " << name << "from" << QThread::currentThread();
    //}
}

void proc(int& a)
{
    qDebug() << "i am child thread, in param = " << a;
    //qDebug() << "子线程中显示子线程id为" << std::this_thread::get_id() << endl;
}

void SoftRasterizer::paintEvent(QPaintEvent*)
{
    //int a = 9;
    //int b = 4;
    //std::thread th2(proc, std::ref(a));//第一个参数为函数名，第二个参数为该函数的第一个参数，如果该函数接收多个参数就依次写在后面。此时线程开始执行。
    //std::thread th3(proc, std::ref(b));//第一个参数为函数名，第二个参数为该函数的第一个参数，如果该函数接收多个参数就依次写在后面。此时线程开始执行。
    //qDebug() << "i am father thread";
    ////此处省略多行，不要在创建完线程后马上join,应该在程序结束前join
    //th2.join();//此时主线程被阻塞直至子线程执行结束。
    //th3.join();//此时主线程被阻塞直至子线程执行结束。

    //QFuture<void> fut1 = run(func, QString("Thread 1"));
    //QFuture<void> fut2 = run(func, QString("Thread 2"));

    //fut1.waitForFinished();
    //fut2.waitForFinished();
    //return;

    clock_t currTime = clock();
    deltaTime = float(currTime - lastFrameTime) / CLOCKS_PER_SEC;
    lastFrameTime = currTime;

    clock_t start, end;
    start = clock();
    scene.render(frontBuffer);
    label.setPixmap(QPixmap::fromImage(img));

 //   updateShadowConstantBuffer();
 //   updateConstantBuffer();

 //   r->renderSingleFrame(frontBuffer);
	//label.setPixmap(QPixmap::fromImage(img));
	//end = clock();
 //   qDebug() << CLOCKS_PER_SEC / double(end - start) << " FPS";
 //   qDebug() << double(end - start) / CLOCKS_PER_SEC  << " s";
 //   return;

 //   auto& attrib = reader.GetAttrib();
 //   auto& shapes = reader.GetShapes();
 //   auto& materials = reader.GetMaterials();



	//clear();

 //   // On CPU and simple scene, earlyZ may slow down render speed
 //   if (earlyZ)
	//	renderDepthMap();

 //   renderShadowMap();

 //   render();

	//label.setPixmap(QPixmap::fromImage(img));


	end = clock();
    qDebug() << CLOCKS_PER_SEC / double(end - start) << " FPS";
    qDebug() << double(end - start) / CLOCKS_PER_SEC  << " s";
}

void SoftRasterizer::keyPressEvent(QKeyEvent* e)
{
    const float speed = 0.25;
    
    float step = speed;

    Vector3f offset = Vector3f::Zero();
    //Vector3f front = cam.z;
    //Vector3f right = cam.x;
    //Vector3f up = cam.y;
    switch (e->key())
    {
    case Qt::Key_W:
        //offset = front;
        offset.z() = 1;
        break;
    case Qt::Key_S:
        //offset =  -front;
        offset.z() = -1;
        break;
    case Qt::Key_A:
        //offset =  -right;
        offset.x() = -1;
        break;
    case Qt::Key_D:
        //offset =  right;
        offset.x() = 1;
        break;
    case Qt::Key_Q:
        //offset =  -up;
        offset.y() = -1;
        break;
    case Qt::Key_E:
        //offset =  up;
        offset.y() = 1;
        break;
    case Qt::Key_Space:
        scene.resetCamera();
        scene.resetModelMatrix();
		//resetCamera();
        //resetModelMatrix();
		//resetShadowCamera();
		break;
    }
    offset *= step;

    scene.updateCameraPosition(offset);
 //   cam.setPosition(cam.pos + offset);
 //   cam.setTarget(cam.target + offset);
	//cam.updateTransform();

    //shadowCam.setPosition(shadowCam.pos + offset);
    //shadowCam.setTarget(shadowCam.target + offset);
    //shadowCam.updateTransform();
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
    rotationAxis.normalize();

    // rotate camera
    if (e->buttons() & Qt::LeftButton) {
        scene.updateCameraRotation(rotationAxis, rotationRadian);

		//cam.rotate(rotationAxis, -rotationRadian);
		//cam.rotateAroundY(rotationRadians.x());
		//cam.rotateAroundX(rotationRadians.y());
		//cam.updateTransform();

		//Vector3f viewDir = cam.target - cam.pos;
		//qDebug("viewDir = (%f, %f, %f)", viewDir.x(), viewDir.y(), viewDir.z());
		//shadowCam.rotate(rotationAxis, rotationRadian);
		//shadowCam.updateTransform();
    }
    // rotate object
    else if (e->buttons() & Qt::RightButton) {
        Eigen::AngleAxisf rotationVector(-rotationRadian, rotationAxis);
        Matrix3f rotationMatrix = rotationVector.matrix();

        scene.updateObjectToWorld(rotationMatrix);
        //modelMatrix.topLeftCorner(3, 3) = rotationMatrix * modelMatrix.topLeftCorner(3, 3);
    }

    lastFrameMousePos = currFrameMousePos;
}

void SoftRasterizer::wheelEvent(QWheelEvent* e)
{
    const float speed = 0.01;
    float step = speed * e->angleDelta().y();

    scene.updateCameraVfov(step);
    //float fov = cam.vfov;
    //fov -= step;
    //fov = std::max(fov, 0.0f);
    //fov = std::min(fov, 180.0f);
    //cam.setVFov(fov);
    //cam.updateProjection();

    //shadowCam.setVFov(fov);
    //shadowCam.updateProjection();
}


//void SoftRasterizer::resetCamera()
//{
//    cam.setPosition(Vector3f(0, 0, -4.0));
//    cam.setTarget(Vector3f(0, 0, 0));
//    cam.setUp(Vector3f(0, 1, 0));
//
//    cam.setVFov(90);
//    cam.setRatio(float(imageWidth) / float(imageHeight));
//    cam.setNear(0.1);
//    cam.setFar(1000);
//}

//void SoftRasterizer::resetModelMatrix()
//{
//    modelMatrix = Matrix4f::Identity();
//}

//void SoftRasterizer::resetShadowCamera()
//{
//    shadowCam.setPosition(mainLight.position);
//    shadowCam.setTarget(mainLight.position - mainLight.direction);
//    shadowCam.setUp(Vector3f(0, 0, 1));
//
//    shadowCam.setLeft(-2);
//    shadowCam.setRight(2);
//    shadowCam.setBottom(-2);
//    shadowCam.setTop(2);
//    //shadowCam.setVFov(160);
//    //shadowCam.setRatio(1);
//    shadowCam.setNear(0.1);
//    shadowCam.setFar(50);
//
//    //shadowCam.setPosition(Vector3f(0, 0, -1.0));
//    //shadowCam.setTarget(Vector3f(0, 0, 0));
//    //shadowCam.setUp(Vector3f(0, 1, 0));
//
//    //shadowCam.setVFov(90);
//    //shadowCam.setRatio(1);
//    //shadowCam.setNear(0.1);
//    //shadowCam.setFar(50);
//}

//void SoftRasterizer::updateConstantBuffer()
//{
//    cb.cameraPosition = cam.pos;
//    cb.objectToWorld = modelMatrix;
//    cb.worldToCamera = cam.getWorldToCamera();
//    cb.projection = cam.getProjection();
//
//    cb.lightShadowViewMatrix = shadowCb.worldToCamera;
//    cb.lightShadowProjMatrix = shadowCb.projection;
//    //cb.shadowMap = shadowMaps[0].data();
//    cb.lightShadowWidth = shadowWidth;
//}

//void SoftRasterizer::updateShadowConstantBuffer()
//{
//    shadowCb.cameraPosition = mainLight.position;
//    shadowCb.objectToWorld = modelMatrix;
//    shadowCb.worldToCamera = shadowCam.getWorldToCamera();
//    shadowCb.projection = shadowCam.getProjection();
//
//    //shadowCb.cameraPosition = shadowCam.pos;
//    //shadowCb.objectToWorld = modelMatrix;
//    //shadowCb.worldToCamera = shadowCam.getWorldToCamera();
//    //shadowCb.projection = shadowCam.getProjection();
//}

//FragmentInput SoftRasterizer::barycentricInterpolation(const FragmentInput* vertices, Vector3f& baryCoord, Vector3f& perspectiveBaryCoord)
//{
//    FragmentInput output;
//    output.positionWS = perspectiveBaryCoord.x() * vertices[0].positionWS + perspectiveBaryCoord.y() * vertices[1].positionWS + perspectiveBaryCoord.z() * vertices[2].positionWS;
//    output.positionCS = perspectiveBaryCoord.x() * vertices[0].positionCS + perspectiveBaryCoord.y() * vertices[1].positionCS + perspectiveBaryCoord.z() * vertices[2].positionCS;
//    output.texCoord = perspectiveBaryCoord.x() * vertices[0].texCoord + perspectiveBaryCoord.y() * vertices[1].texCoord + perspectiveBaryCoord.z() * vertices[2].texCoord;
//    output.shadowCoord = perspectiveBaryCoord.x() * vertices[0].shadowCoord + perspectiveBaryCoord.y() * vertices[1].shadowCoord + perspectiveBaryCoord.z() * vertices[2].shadowCoord;
//    output.normalWS = perspectiveBaryCoord.x() * vertices[0].normalWS + perspectiveBaryCoord.y() * vertices[1].normalWS + perspectiveBaryCoord.z() * vertices[2].normalWS;
//    output.normalWS.normalize();
//    return output;
//}

//void SoftRasterizer::renderDepthMap()
//{
//    auto& attrib = reader.GetAttrib();
//    auto& shapes = reader.GetShapes();
//    auto& materials = reader.GetMaterials();
//
//    int counter = 0;
//    // Loop over shapes
//    for (size_t s = 0; s < shapes.size(); s++) {
//        // Loop over faces(polygon)
//        size_t index_offset = 0;
//        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
//            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
//            Triangle tri;
//
//            // Loop over vertices in the face.
//            for (size_t v = 0; v < fv; v++) {
//                // access to vertex
//                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
//                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
//                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
//                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
//
//                tri.v[v] = Vector4f(vx, vy, vz, 1.0);
//            }
//            index_offset += fv;
//            counter += drawTriangleDepth(depthBuffer.data(), tri);
//        }
//    }
//}
//
//void SoftRasterizer::renderShadowMap()
//{
//
//    Vector4f a(-2, -1, -2, 1);
//    Vector4f b(2, -1, -2, 1);
//    Vector4f c(-2, -1, 2, 1);
//    Vector4f d(2, -1, 2, 1);
//    Vector3f n(0, 1, 0);
//    std::array<Vector3f, 3> normals{ n, n, n };
//
//    Triangle planeTri0(a, b, c);
//    Triangle planeTri1(c, b, d);
//    planeTri0.setNormals(normals);
//    planeTri1.setNormals(normals);
//    drawTriangleShadow(shadowMaps[0].data(), planeTri0);
//    drawTriangleShadow(shadowMaps[0].data(), planeTri1);
//
//    auto& attrib = reader.GetAttrib();
//    auto& shapes = reader.GetShapes();
//    auto& materials = reader.GetMaterials();
//
//    //qDebug() << "shapes.size() = " << shapes.size();
//    int counter = 0;
//    // Loop over shapes
//    for (size_t s = 0; s < shapes.size(); s++) {
//        // Loop over faces(polygon)
//        size_t index_offset = 0;
//        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
//            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
//            Triangle tri;
//
//            // Loop over vertices in the face.
//            for (size_t v = 0; v < fv; v++) {
//                // access to vertex
//                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
//                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
//                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
//                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
//
//                tri.v[v] = Vector4f(vx, vy, vz, 1.0);
//            }
//            index_offset += fv;
//            counter += drawTriangleShadow(shadowMaps[0].data(), tri);
//        }
//    }
//   // std::vector<uchar> temp(bufferSize);
//   // for (int i = 0; i < shadowWidth; ++i) {
//   //     for (int j = 0; j < shadowWidth; ++j) {
//   //         int index = i + j * imageWidth;
//   //         int pixelIndex = i + j * shadowWidth;
//			//temp[4 * index] = uchar(shadowMaps[0][pixelIndex] * 255.99f);
//			//temp[4 * index + 3] = 255;
//   //     }
//   // }
//   // memcpy(frontBuffer, temp.data(), bufferSize);
//}

//void SoftRasterizer::render()
//{
//    auto& attrib = reader.GetAttrib();
//    auto& shapes = reader.GetShapes();
//    auto& materials = reader.GetMaterials();
//
//    uchar* backBuffer = new uchar[bufferSize];
//    memset(backBuffer, 255, bufferSize);
//
//    Vector4f a(-2, -1, -2, 1);
//    Vector4f b(2, -1, -2, 1);
//    Vector4f c(-2, -1, 2, 1);
//    Vector4f d(2, -1, 2, 1);
//    Vector3f n(0, 1, 0);
//    std::array<Vector3f, 3> normals{n, n, n};
//
//    Triangle planeTri0(a, b, c);
//    Triangle planeTri1(c, b, d);
//    planeTri0.setNormals(normals);
//    planeTri1.setNormals(normals);
//    drawTriangle(backBuffer, planeTri0);
//    drawTriangle(backBuffer, planeTri1);
//
//
//
//    int counter = 0;
//    // Loop over shapes
//    for (size_t s = 0; s < shapes.size(); s++) {
//        // Loop over faces(polygon)
//        size_t index_offset = 0;
//        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
//            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
//            Triangle tri;
//
//            // Loop over vertices in the face.
//            for (size_t v = 0; v < fv; v++) {
//                // access to vertex
//                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
//                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
//                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
//                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
//
//                tri.v[v] = Vector4f(vx, vy, vz, 1.0);
//                //qDebug() << vx << ", " << vy << ", " << vz;
//
//                // Check if `normal_index` is zero or positive. negative = no normal data
//                if (idx.normal_index >= 0) {
//                    tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
//                    tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
//                    tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];
//
//                    tri.normals[v] = Vector3f(nx, ny, nz).normalized();
//                }
//
//                // Check if `texcoord_index` is zero or positive. negative = no texcoord data
//                if (idx.texcoord_index >= 0) {
//                    tinyobj::real_t tx = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
//                    tinyobj::real_t ty = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];
//
//                    //qDebug() << tx << ", " << ty;
//                    tri.texCoords[v] = Vector2f(tx, ty);
//                }
//
//                // Optional: vertex colors
//                tinyobj::real_t red = attrib.colors[3 * size_t(idx.vertex_index) + 0];
//                tinyobj::real_t green = attrib.colors[3 * size_t(idx.vertex_index) + 1];
//                tinyobj::real_t blue = attrib.colors[3 * size_t(idx.vertex_index) + 2];
//
//                //qDebug() << red << ", " << green << ", " << blue;
//                tri.colors[v] = Vector4f(red, green, blue, 255);
//            }
//            index_offset += fv;
//
//            // per-face material
//            shapes[s].mesh.material_ids[f];
//
//            counter += drawTriangle(backBuffer, tri);
//        }
//    }
//    qDebug() << "run fragment shader " << counter << " times";
//
//    memcpy(frontBuffer, backBuffer, bufferSize);
//    delete[] backBuffer;
//}

