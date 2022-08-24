#ifndef FAST_RENDERER_H
#define FAST_RENDERER_H

#include "tiny_obj_loader.h"
#include "Eigen/Core"

#include "Triangle.h"
#include "Camera.h"
#include "Transform.h"
#include "ConstantBuffer.h"
#include "Shader.h"


#include <functional>
#include <vector>
#include <thread>

#include <QtConcurrent/QtConcurrent>
using namespace QtConcurrent;


using namespace Eigen;

struct ConstantBuffer;
//struct VertexInput;
//struct FragmentInput;
class Camera;

//struct FragmentInfo
//{
//
//};

class FastRenderer
{
    friend class Camera;
public:
    FastRenderer();
    void renderColorBuffer();
    void renderDepthBuffer();
    void clearColorBuffer();
    void clearDepthBuffer();

    //void bindAll(
    //    int w, int h,
    //    Camera* c,
    //    const tinyobj::ObjReader* r,
    //    uchar* colorBuffer, float* depthBuffer, float* shadowMap,
    //    ConstantBuffer* constantBuffer
    //);
    void setSize(int w, int h);

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


    void bindColorBuffer(uchar* cb, int width, int height);
    void disposeColorBuffer();

    void bindDepthBuffer(float* db, int width, int height);
    void disposeDepthBuffer();

    void bindShadowMap(float* sm, int width);
    void disposeShadowMap();

    void bindConstantBuffer(const ConstantBuffer& buffer);
    void disposeConstantBuffer();

//private:
    //typedef void (FastRenderer::*PR)(int index, float depth, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord);
    //void renderBuffer(PR pixelRenderer);
    void renderBuffer(int renderType);

    int renderSingleTriangle(const Triangle& tri, int renderType, int triIndex);
    static Vector4f screenMapping(int width, int height, Vector4f clipSpaceCoord);
    //static FragmentInput barycentricInterpolation(const FragmentInput* vertices, const Vector3f& perspectiveBaryCoord);
    static void barycentricInterpolation(FragmentInput& output, const FragmentInput* vertices, const Vector3f& baryCoord, const Vector3f& perspectiveBaryCoord);

    void depthRenderer(int index, float depth, const FragmentInput* fi, const Vector3f& baryCoord, const Vector3f& perspectiveBaryCoord);
    void colorRenderer(int index, float depth, const FragmentInput* fi, const Vector3f& baryCoord, const Vector3f& perspectiveBaryCoord);

    int screenWidth;
    int screenHeight;

    bool enableEarlyZ;
    bool enableShadow;


    const tinyobj::ObjReader* reader = nullptr;

    float* shadowMap;
    float* depthBuffer;
    uchar* colorBuffer;

    ConstantBuffer constantBuffer;

    QPoint lastFrameMousePos;
    clock_t lastFrameTime;
    float deltaTime;

    const std::vector<Vector3i>* triangles;
    const std::vector<Vector3f>* vertices;
    const std::vector<Vector3f>* normals;
    const std::vector<Vector2f>* texCoords;
    const std::vector<Vector3f>* colors;
    const std::vector<Triangle>* completeTris;

    //std::vector<>
};

#endif // !FAST_RENDERER_H
