#include "FastRenderer.h"

QMutex mutex;

FastRenderer::FastRenderer() :
    screenWidth(0), screenHeight(0),
    enableEarlyZ(false), enableShadow(true),
    shadowMap(nullptr),
    depthBuffer(nullptr),
    colorBuffer(nullptr),
    deltaTime(0),
    lastFrameTime(0)
{
    //parallel_for(0, 10, [](int num) {});
}

void FastRenderer::renderColorBuffer()
{
    clearDepthBuffer();
    clearColorBuffer();

    if (enableEarlyZ) {
        renderBuffer(0);
    }
    

    renderBuffer(1);
}

void FastRenderer::renderDepthBuffer()
{
    clearDepthBuffer();
	renderBuffer(0);
}

void FastRenderer::clearColorBuffer()
{
    int pixelSize = screenWidth * screenHeight;
    int colorBufferSize = pixelSize * 4;

    std::fill(colorBuffer, colorBuffer + colorBufferSize, 255);
}

void FastRenderer::clearDepthBuffer()
{
    int pixelSize = screenWidth * screenHeight;
    std::fill(depthBuffer, depthBuffer + pixelSize, 0);
}


//void FastRenderer::bindAll(int w, int h, Camera* c, const tinyobj::ObjReader* r, uchar* colorBuffer, float* depthBuffer, float* shadowMap, ConstantBuffer* constantBuffer) {
//    setSize(w, h);
//    bindReader(r);
//    bindColorBuffer(colorBuffer);
//    bindDepthBuffer(depthBuffer);
//    bindShadowMap(shadowMap);
//    bindConstantBuffer(constantBuffer);
//}

void FastRenderer::setSize(int w, int h) { screenWidth = w; screenHeight = h; }

void FastRenderer::bindReader(const tinyobj::ObjReader* r) { reader = r; }

void FastRenderer::disposeReader()
{
    reader = nullptr;
}

void FastRenderer::bindData(
    const std::vector<Vector3i>* triangles,
    const std::vector<Vector3f>* vertices,
    const std::vector<Vector3f>* normals,
    const std::vector<Vector2f>* texCoords,
    const std::vector<Vector3f>* colors,
    const std::vector<Triangle>* completeTris
)
{
    this->triangles = triangles;
    this->vertices = vertices;
    this->normals = normals;
    this->texCoords = texCoords;
    this->colors = colors;
    this->completeTris = completeTris;
}

void FastRenderer::disposeData()
{
}

void FastRenderer::bindColorBuffer(uchar* cb, int width, int height)
{
    colorBuffer = cb; screenWidth = width; screenHeight = height;
}

void FastRenderer::disposeColorBuffer()
{
    colorBuffer = nullptr;
}

void FastRenderer::bindDepthBuffer(float* db, int width, int height)
{
    depthBuffer = db;
    screenWidth = width;
    screenHeight = height;

}

void FastRenderer::disposeDepthBuffer()
{
    depthBuffer = nullptr;
}

void FastRenderer::bindShadowMap(float* sm, int width) { shadowMap = sm;}

void FastRenderer::disposeShadowMap()
{
    shadowMap = nullptr;
}

void FastRenderer::bindConstantBuffer(const ConstantBuffer& buffer) { constantBuffer = buffer; }

void FastRenderer::disposeConstantBuffer()
{
    //constantBuffer = nullptr;
}

void FastRenderer::renderBuffer(int renderType)
{
    auto& attrib = reader->GetAttrib();
    auto& shapes = reader->GetShapes();
    auto& materials = reader->GetMaterials();


    float groundy = -0.736784;
    Vector4f a(-2, groundy, -2, 1);
    Vector4f b(2, groundy, -2, 1);
    Vector4f c(-2, groundy, 2, 1);
    Vector4f d(2, groundy, 2, 1);
    Vector3f n(0, 1, 0);
    std::array<Vector3f, 3> planeNormal{ n, n, n };

    Triangle planeTri0(a, b, c);
    Triangle planeTri1(c, b, d);
    planeTri0.setNormals(planeNormal);
    planeTri1.setNormals(planeNormal);

    renderSingleTriangle(planeTri0, renderType, -2);
    renderSingleTriangle(planeTri1, renderType, -1);

    //std::vector<std::thread> pool;
    //int numThreads = 8;
    //for (int k = 0; k < numThreads; ++k) {
    //    pool.push_back(std::thread(
    //        [&]() {
    //            for (int i = k; i < triangles->size(); i += numThreads) {
    //                Vector3i indices = (*triangles)[i];
    //                Triangle tri;
    //                int x = indices.x();
    //                int y = indices.y();
    //                int z = indices.z();

    //                tri.v[0] = Vector4f((*vertices)[x].x(), (*vertices)[x].y(), (*vertices)[x].z(), 1);
    //                tri.v[1] = Vector4f((*vertices)[y].x(), (*vertices)[y].y(), (*vertices)[y].z(), 1);
    //                tri.v[2] = Vector4f((*vertices)[z].x(), (*vertices)[z].y(), (*vertices)[z].z(), 1);

    //                tri.normals[0] = (*normals)[x];
    //                tri.normals[1] = (*normals)[y];
    //                tri.normals[2] = (*normals)[z];

    //                renderSingleTriangle(tri, renderType);
    //            }
    //        }
    //    ));
    //}

    //for (int i = 0; i < numThreads; ++i) {
    //    pool[i].join();
    //}

    //if (renderType == 1) {
    if (false) {

        std::vector<QFuture<void>> qfs;
        int numThreads = 6;
        for (int k = 0; k < numThreads; ++k) {
            //qDebug("k = %d", k);
            qfs.push_back(
                QtConcurrent::run(
                    [=]() {
                        for (int i = k; i < triangles->size(); i += numThreads) {
                            //Vector3i indices = (*triangles)[i];
                            //Triangle tri;
                            //int x = indices.x();
                            //int y = indices.y();
                            //int z = indices.z();

                            //tri.v[0] = Vector4f((*vertices)[x].x(), (*vertices)[x].y(), (*vertices)[x].z(), 1);
                            //tri.v[1] = Vector4f((*vertices)[y].x(), (*vertices)[y].y(), (*vertices)[y].z(), 1);
                            //tri.v[2] = Vector4f((*vertices)[z].x(), (*vertices)[z].y(), (*vertices)[z].z(), 1);

                            //tri.normals[0] = (*normals)[x];
                            //tri.normals[1] = (*normals)[y];
                            //tri.normals[2] = (*normals)[z];

                            //renderSingleTriangle(tri, renderType, i);
                            renderSingleTriangle((*completeTris)[i], renderType, i);
                        }
                    }
            ));
        }

        for (int i = 0; i < numThreads; ++i) {
            qfs[i].waitForFinished();
        }
    }
    else {
        for (int i = 0; i < triangles->size(); ++i) {
            Vector3i indices = (*triangles)[i];
            Triangle tri;
            int x = indices.x();
            int y = indices.y();
            int z = indices.z();

            tri.v[0] = Vector4f((*vertices)[x].x(), (*vertices)[x].y(), (*vertices)[x].z(), 1);
            tri.v[1] = Vector4f((*vertices)[y].x(), (*vertices)[y].y(), (*vertices)[y].z(), 1);
            tri.v[2] = Vector4f((*vertices)[z].x(), (*vertices)[z].y(), (*vertices)[z].z(), 1);

            tri.normals[0] = (*normals)[x];
            tri.normals[1] = (*normals)[y];
            tri.normals[2] = (*normals)[z];

            renderSingleTriangle(tri, renderType, i);
        }
    }
}


Vector4f FastRenderer::screenMapping(int width, int height, Vector4f clipSpaceCoord)
{
    float invW = 1.0 / clipSpaceCoord.w();
    Vector4f NDC(
        clipSpaceCoord.x() * invW,
        clipSpaceCoord.y() * invW,
        clipSpaceCoord.z() * invW,
        invW
    );
    Vector4f screenSpaceCoord(
        (NDC.x() + 1) * width * 0.5,
        (NDC.y() + 1) * height * 0.5,
        NDC.z(),
        NDC.w()
    );
    return screenSpaceCoord;
}

void FastRenderer::barycentricInterpolation(FragmentInput& output, const FragmentInput* vertices, const Vector3f& baryCoord, const Vector3f& perspectiveBaryCoord)
{
    output.positionWS = perspectiveBaryCoord.x() * vertices[0].positionWS + perspectiveBaryCoord.y() * vertices[1].positionWS + perspectiveBaryCoord.z() * vertices[2].positionWS;
    output.positionCS = perspectiveBaryCoord.x() * vertices[0].positionCS + perspectiveBaryCoord.y() * vertices[1].positionCS + perspectiveBaryCoord.z() * vertices[2].positionCS;
    //output.texCoord = perspectiveBaryCoord.x() * vertices[0].texCoord + perspectiveBaryCoord.y() * vertices[1].texCoord + perspectiveBaryCoord.z() * vertices[2].texCoord;
    output.shadowCoord = perspectiveBaryCoord.x() * vertices[0].shadowCoord + perspectiveBaryCoord.y() * vertices[1].shadowCoord + perspectiveBaryCoord.z() * vertices[2].shadowCoord;
    //output.shadowCoord = baryCoord.x() * vertices[0].shadowCoord + baryCoord.y() * vertices[1].shadowCoord + baryCoord.z() * vertices[2].shadowCoord;
    //output.shadowCoord.head(2) =
    //    perspectiveBaryCoord.x() * vertices[0].shadowCoord.head(2) +
    //    perspectiveBaryCoord.y() * vertices[1].shadowCoord.head(2) +
    //    perspectiveBaryCoord.z() * vertices[2].shadowCoord.head(2);
    //output.shadowCoord.tail(2) =
    //    baryCoord.x() * vertices[0].shadowCoord.tail(2) +
    //    baryCoord.y() * vertices[1].shadowCoord.tail(2) +
    //    baryCoord.z() * vertices[2].shadowCoord.tail(2);

    output.normalWS = perspectiveBaryCoord.x() * vertices[0].normalWS + perspectiveBaryCoord.y() * vertices[1].normalWS + perspectiveBaryCoord.z() * vertices[2].normalWS;
    output.normalWS.normalize();
}

void FastRenderer::depthRenderer(int index, float depth, const FragmentInput* fi, const Vector3f& baryCoord, const Vector3f& perspectiveBaryCoord)
{
	//if (depthBuffer[index] > 0)
		//qDebug("depth = %f, depthBuffer = %f", depth, depthBuffer[index]);
    depthBuffer[index] = depth;
}

//void FastRenderer::shadowRenderer(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord)
//{
//    *depthIndex = depth;
//}

void FastRenderer::colorRenderer(int index, float depth, const FragmentInput* fi, const Vector3f& baryCoord, const Vector3f& perspectiveBaryCoord)
{
    depthBuffer[index] = depth;
    //FragmentInput input = barycentricInterpolation(fi, perspectiveBaryCoord);
    FragmentInput input;
    barycentricInterpolation(input, fi, baryCoord, perspectiveBaryCoord);

    //Vector3f color = constantBuffer->material->fragmentShader(*constantBuffer, constantBuffer->material->shaderProps, input);

    //qDebug("lightShadowProjMatrix = ");
    //MyTransform::print(constantBuffer.lightShadowProjMatrix);
    //qDebug("lightShadowViewMatrix = ");
    //MyTransform::print(constantBuffer.lightShadowViewMatrix);

    Vector3f color = constantBuffer.material->fragmentShader(constantBuffer, constantBuffer.material->shaderProps, input);

    color.x() = std::min(color.x(), 1.0f);
    color.y() = std::min(color.y(), 1.0f);
    color.z() = std::min(color.z(), 1.0f);

    int colorIndex = index * 4;
    colorBuffer[colorIndex + 0] = color.x() * 255.99;
    colorBuffer[colorIndex + 1] = color.y() * 255.99;
    colorBuffer[colorIndex + 2] = color.z() * 255.99;
    colorBuffer[colorIndex + 3] = 255.99;
}

int FastRenderer::renderSingleTriangle(const Triangle& tri, int renderType, int triIndex)
{
    Triangle screenSpaceTri;

    FragmentInput fi[3];
    for (int i = 0; i < 3; ++i) {
        VertexInput vi;
        vi.positionOS = tri.v[i];
        vi.normalOS = tri.normals[i];
        fi[i] = constantBuffer.material->vertexShader(constantBuffer, constantBuffer.material->shaderProps, vi);
    }

   // if (triIndex == -2) {
   //     if (renderType == 0) {
			//for (int i = 0; i < 3; ++i) {
			//	//qDebug("tri.v[%d] = (%f, %f, %f, %f)", i, tri.v[i].x(), tri.v[i].y(), tri.v[i].z(), tri.v[i].w());
			//	qDebug("fi[%d].positionCS = (%f, %f, %f, %f)", i, fi[i].positionCS.x(), fi[i].positionCS.y(), fi[i].positionCS.z(), fi[i].positionCS.w());
   //             Vector4f temp;
   //             temp.head(2) = fi[i].positionCS.head(2) / fi[i].positionCS.w() * 0.5 + Vector2f(0.5, 0.5);
   //             temp.z() = fi[i].positionCS.z() / fi[i].positionCS.w();
			//	temp.w() = 1.0f / fi[i].positionCS.w();
			//	qDebug("fi[%d].screenSpace = (%f, %f, %f, %f)", i, temp.x(), temp.y(), temp.z(), temp.w());
			//	//qDebug("fi[%d].shadowCoord = (%f, %f, %f, %f)", i, fi[i].shadowCoord.x(), fi[i].shadowCoord.y(), fi[i].shadowCoord.z(), fi[i].shadowCoord.w());
			//}
   //     }
   //     if (renderType == 1) {
			//for (int i = 0; i < 3; ++i) {
			//	//qDebug("tri.v[%d] = (%f, %f, %f, %f)", i, tri.v[i].x(), tri.v[i].y(), tri.v[i].z(), tri.v[i].w());
			//	//qDebug("fi[%d].positionCS = (%f, %f, %f, %f)", i, fi[i].positionCS.x(), fi[i].positionCS.y(), fi[i].positionCS.z(), fi[i].positionCS.w());
			//	qDebug("fi[%d].shadowCoord = (%f, %f, %f, %f)", i, fi[i].shadowCoord.x(), fi[i].shadowCoord.y(), fi[i].shadowCoord.z(), fi[i].shadowCoord.w());
			//}
   //     }
   // }

    //for (int i = 0; i < 3; ++i) {
    //    qDebug("tri.v[i] = (%f, %f, %f, %f)", i, tri.v[i].x(), tri.v[i].y(), tri.v[i].z(), tri.v[i].w());
    //    qDebug("fi[%d].positionCS = (%f, %f, %f, %f)", i, fi[i].positionCS.x(), fi[i].positionCS.y(), fi[i].positionCS.z(), fi[i].positionCS.w());
    //}

    if (constantBuffer.cameraType == 0) {
        Vector3f viewDirs[3];
        viewDirs[0] = constantBuffer.cameraPosition - fi[0].positionWS.head(3);
        viewDirs[1] = constantBuffer.cameraPosition - fi[1].positionWS.head(3);
        viewDirs[2] = constantBuffer.cameraPosition - fi[2].positionWS.head(3);
        if (viewDirs[0].dot(fi[0].normalWS) < 0 &&
            viewDirs[1].dot(fi[1].normalWS) < 0 &&
            viewDirs[2].dot(fi[2].normalWS) < 0
            ) {
            return 0;
        }
    }
    else if (constantBuffer.cameraType == 1) {
        Vector3f viewDir = constantBuffer.cameraPosition - constantBuffer.worldToCamera.block(2, 0, 1, 3).transpose();
        if (viewDir.dot(fi[0].normalWS) < 0 &&
            viewDir.dot(fi[1].normalWS) < 0 &&
            viewDir.dot(fi[2].normalWS) < 0
            ) {
            return 0;
        }
    }


    if (fi[0].positionCS.w() < 0 || fi[1].positionCS.w() < 0 || fi[2].positionCS.w() < 0) {
        return 0;
    }


    screenSpaceTri.v[0] = screenMapping(screenWidth, screenHeight, fi[0].positionCS);
    screenSpaceTri.normals[0] = fi[0].normalWS;

    screenSpaceTri.v[1] = screenMapping(screenWidth, screenHeight, fi[1].positionCS);
    screenSpaceTri.normals[1] = fi[1].normalWS;

    screenSpaceTri.v[2] = screenMapping(screenWidth, screenHeight, fi[2].positionCS);
    screenSpaceTri.normals[2] = fi[2].normalWS;

    screenSpaceTri.updateInfo();

    float minx = screenWidth, miny = screenHeight;
    float maxx = 0, maxy = 0;
    for (int i = 0; i < 3; i++) {
        minx = screenSpaceTri.v[i].x() < minx ? screenSpaceTri.v[i].x() : minx;
        maxx = screenSpaceTri.v[i].x() > maxx ? screenSpaceTri.v[i].x() : maxx;

        miny = screenSpaceTri.v[i].y() < miny ? screenSpaceTri.v[i].y() : miny;
        maxy = screenSpaceTri.v[i].y() > maxy ? screenSpaceTri.v[i].y() : maxy;
    }

    int iminx = minx;
    int imaxx = ceil(maxx);
    int iminy = miny;
    int imaxy = ceil(maxy);

    iminx = std::max(0, iminx);
    imaxx = std::min(screenWidth - 1, imaxx);
    iminy = std::max(0, iminy);
    imaxy = std::min(screenHeight - 1, imaxy);

    //qDebug("iminx = %d, imaxx = %d, iminy = %d, imaxy = %d", iminx, imaxx, iminy, imaxy);

    Vector3f baryCoord(0, 0, 0);
    Vector3f perspectiveBaryCoord(0, 0, 0);

    for (int x = iminx; x <= imaxx; x++) {
        for (int y = iminy; y <= imaxy; y++) {
            int pixelIndex = x + (screenHeight - 1 - y) * screenWidth;
			//depthBuffer[pixelIndex] = 1;
            //qDebug("depthBuffer = %p", depthBuffer);
            if (screenSpaceTri.pointInsideTriangle(x + 0.5, y + 0.5, baryCoord, perspectiveBaryCoord)) {
                // 1/z, thus shouldn't use perspective interpolation
                float depth = baryCoord.x() * screenSpaceTri.v[0].z() + baryCoord.y() * screenSpaceTri.v[1].z() + baryCoord.z() * screenSpaceTri.v[2].z();

                if (depth > 0 && depth < 1 && depth >= depthBuffer[pixelIndex]) {
                    //if (depthBuffer[pixelIndex] > 0)
						//qDebug("depth = %f, depthBuffer = %f", depth, depthBuffer[pixelIndex]);
                    if (renderType == 0) depthRenderer(pixelIndex, depth, fi, baryCoord, perspectiveBaryCoord);
                    else colorRenderer(pixelIndex, depth, fi, baryCoord, perspectiveBaryCoord);
                }
            }
        }
    }


    return 0;
}
