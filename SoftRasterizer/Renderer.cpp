//#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "Renderer.h"

void Renderer::renderSingleFrame(uchar* frontBuffer)
{
	int colorBufferSize = pixelSize * 4;
    //uchar* colorBuffer = new uchar[colorBufferSize];
    //float* depthBuffer = new float[pixelSize];
    //float* shadowMap = new float[shadowWidth * shadowWidth];
    //std::fill(colorBuffer, colorBuffer + colorBufferSize, 255);
    //std::fill(depthBuffer, depthBuffer + pixelSize, 0);
    //std::fill(shadowMap, shadowMap + shadowWidth * shadowWidth, 0);

    std::vector<uchar> colorBuffer(colorBufferSize, 255);
    std::vector<float> depthBuffer(pixelSize, 0);
    std::vector<float> shadowMap(shadowWidth * shadowWidth, 0);


	if (enableShadow) {
		//renderBuffer(shadowMap, colorBuffer, shadowMaterial, shadowConstantBuffer, depthRenderer);
		renderBuffer(shadowWidth, shadowWidth, shadowMap.data(), colorBuffer.data(), shadowMaterial, shadowConstantBuffer, depthRenderer);
	}

  //   std::vector<uchar> temp(colorBufferSize);
	 //for (int i = 0; i < shadowWidth; ++i) {
		// for (int j = 0; j < shadowWidth; ++j) {
		//	 int index = i + j * screenWidth;
		//	 int pixelIndex = i + j * shadowWidth;
		//	 temp[4 * index] = uchar(shadowMap[pixelIndex] * 255.99f);
  //           //qDebug("shadowMap[%d] = %f", pixelIndex, shadowMap[pixelIndex]);
		//	 temp[4 * index + 3] = 255;
		// }
	 //}
	 //memcpy(frontBuffer, temp.data(), colorBufferSize);
  //   return;

	if (enableEarlyZ) {
        //renderBuffer(depthBuffer, colorBuffer, material, materialConstantBuffer, depthRenderer);
        renderBuffer(screenWidth, screenHeight ,depthBuffer.data(), colorBuffer.data(), material, materialConstantBuffer, depthRenderer);
	}

    //renderBuffer(depthBuffer, colorBuffer, material, materialConstantBuffer, colorRenderer);
    materialConstantBuffer->shadowMap = shadowMap.data();
    renderBuffer(screenWidth, screenHeight, depthBuffer.data(), colorBuffer.data(), material, materialConstantBuffer, colorRenderer);

	memcpy(frontBuffer, colorBuffer.data(), colorBufferSize);
	//memcpy(frontBuffer, colorBuffer, colorBufferSize);
    //delete[] colorBuffer;
    //delete[] depthBuffer;
    //delete[] shadowMap;
}

void Renderer::renderBuffer(int width, int height, float* depthBuffer, uchar* colorBuffer, const Material* mat, const ConstantBuffer* cb, PR pixelRenderer)
{
    auto& attrib = reader->GetAttrib();
    auto& shapes = reader->GetShapes();
    auto& materials = reader->GetMaterials();

    int bufferSize = pixelSize * 4;

    Vector4f a(-2, -1, -2, 1);
    Vector4f b(2, -1, -2, 1);
    Vector4f c(-2, -1, 2, 1);
    Vector4f d(2, -1, 2, 1);
    Vector3f n(0, 1, 0);
    std::array<Vector3f, 3> normals{ n, n, n };

    Triangle planeTri0(a, b, c);
    Triangle planeTri1(c, b, d);
    planeTri0.setNormals(normals);
    planeTri1.setNormals(normals);

    renderSingleTriangle(width, height, depthBuffer, colorBuffer, planeTri0, mat, cb, pixelRenderer);
    renderSingleTriangle(width, height, depthBuffer, colorBuffer, planeTri1, mat, cb, pixelRenderer);

    //drawTriangle(backBuffer, planeTri0);
    //drawTriangle(backBuffer, planeTri1);

    int counter = 0;
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        //qDebug() << "shapes[s].mesh.num_face_vertices.size() = " << shapes[s].mesh.num_face_vertices.size();
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

            //counter += drawTriangle(backBuffer, tri);
            counter += renderSingleTriangle(width, height, depthBuffer, colorBuffer, tri, mat, cb, pixelRenderer);
        }
    }
}


Vector4f Renderer::screenMapping(int width, int height, Vector4f clipSpaceCoord)
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

FragmentInput Renderer::barycentricInterpolation(const FragmentInput* vertices, const Vector3f& perspectiveBaryCoord)
{
    FragmentInput output;
    output.positionWS = perspectiveBaryCoord.x() * vertices[0].positionWS + perspectiveBaryCoord.y() * vertices[1].positionWS + perspectiveBaryCoord.z() * vertices[2].positionWS;
    output.positionCS = perspectiveBaryCoord.x() * vertices[0].positionCS + perspectiveBaryCoord.y() * vertices[1].positionCS + perspectiveBaryCoord.z() * vertices[2].positionCS;
    output.texCoord = perspectiveBaryCoord.x() * vertices[0].texCoord + perspectiveBaryCoord.y() * vertices[1].texCoord + perspectiveBaryCoord.z() * vertices[2].texCoord;
    output.shadowCoord = perspectiveBaryCoord.x() * vertices[0].shadowCoord + perspectiveBaryCoord.y() * vertices[1].shadowCoord + perspectiveBaryCoord.z() * vertices[2].shadowCoord;
    output.normalWS = perspectiveBaryCoord.x() * vertices[0].normalWS + perspectiveBaryCoord.y() * vertices[1].normalWS + perspectiveBaryCoord.z() * vertices[2].normalWS;
    output.normalWS.normalize();
    return output;
}

void Renderer::depthRenderer(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord)
{
    *depthIndex = depth;
    //qDebug() << "depth depth = " << depth;
}

void Renderer::shadowRenderer(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord)
{
    *depthIndex = depth;
    //qDebug() << "shadow depth = " << depth;
}

void Renderer::colorRenderer(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord)
{
    *depthIndex = depth;
    FragmentInput input = barycentricInterpolation(fi, perspectiveBaryCoord);

    Vector3f color = mat->fragmentShader(*cb, mat->shaderProps, input);

    color.x() = std::min(color.x(), 1.0f);
    color.y() = std::min(color.y(), 1.0f);
    color.z() = std::min(color.z(), 1.0f);

    colorIndex[0] = color.x() * 255.99;
    colorIndex[1] = color.y() * 255.99;
    colorIndex[2] = color.z() * 255.99;
    colorIndex[3] = 255.99;
}

int Renderer::renderSingleTriangle(int width, int height, float* depthBuffer, uchar* colorBuffer, Triangle& tri, const Material* mat, const ConstantBuffer* cb, PR pixelRenderer)
{
    Triangle screenSpaceTri;

    Vector3f a = tri.v[0].head(3);
    Vector3f b = tri.v[1].head(3);
    Vector3f c = tri.v[2].head(3);

    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
    Vector3f avePos = (a + b + c) / 3;

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
        fi[i] = mat->vertexShader(*cb, mat->shaderProps, vi);
    }

    Vector3f viewDir = cam->pos - avePos;
    aveNormal = (fi[0].normalWS + fi[1].normalWS + fi[2].normalWS).normalized();

    if (viewDir.dot(aveNormal) < 0) {
        return 0;
    }

    if (fi[0].positionCS.w() < 0 || fi[1].positionCS.w() < 0 || fi[2].positionCS.w() < 0) {
        return 0;
    }

    screenSpaceTri.v[0] = screenMapping(width, height, fi[0].positionCS);
    screenSpaceTri.normals[0] = fi[0].normalWS;

    screenSpaceTri.v[1] = screenMapping(width, height, fi[1].positionCS);
    screenSpaceTri.normals[1] = fi[1].normalWS;

    screenSpaceTri.v[2] = screenMapping(width, height, fi[2].positionCS);
    screenSpaceTri.normals[2] = fi[2].normalWS;

    screenSpaceTri.updateInfo();

    float minx = width, miny = height;
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
    imaxx = std::min(width - 1, imaxx);
    iminy = std::max(0, iminy);
    imaxy = std::min(height - 1, imaxy);

    Vector3f baryCoord(0, 0, 0);
    Vector3f perspectiveBaryCoord(0, 0, 0);

    for (int x = iminx; x <= imaxx; x++) {
        for (int y = iminy; y <= imaxy; y++) {
            int pixelIndex = x + (height - 1 - y) * width;
            //int startIndex = pixelIndex * 4;
            int startIndex = pixelIndex;
            if (screenSpaceTri.pointInsideTriangle(x + 0.5, y + 0.5, baryCoord, perspectiveBaryCoord)) {
                float depth = baryCoord.x() * screenSpaceTri.v[0].z() + baryCoord.y() * screenSpaceTri.v[1].z() + baryCoord.z() * screenSpaceTri.v[2].z();

                if (depth > 0 && depth < 1 && depth >= depthBuffer[pixelIndex]) {
                    pixelRenderer(depthBuffer + pixelIndex, colorBuffer + pixelIndex * 4, depth, mat, cb, fi, perspectiveBaryCoord);
                }
            }
        }
    }
    return 0;
}

