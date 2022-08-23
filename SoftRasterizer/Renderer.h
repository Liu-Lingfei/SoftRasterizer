//#ifndef RENDERER_H
//#define RENDREER_H
//
//#include "Eigen/Core"
//#include "tiny_obj_loader.h"
//#include "Light.h"
//#include "Material.h"
//#include "Input.h"
//#include "Camera.h"
//#include "Triangle.h"
//
//#include <functional>
//#include <vector>
//
//using namespace Eigen;
//
//class Renderer
//{
//public:
//    Renderer(int width, int height, int shadowWidth, bool earlyZ = false, bool shadow = true)
//        :
//        screenWidth(width), screenHeight(height), shadowWidth(shadowWidth),
//        enableEarlyZ(earlyZ), enableShadow(shadow),
//        pixelSize(width* height),
//        modelMatrix(nullptr),
//        deltaTime(0),
//        lastFrameTime(0)
//    {}
//	void renderSingleFrame(uchar* frontBuffer);
//    void setReader(const tinyobj::ObjReader* r) { reader = r; }
//    void setModelMatrix(const Matrix4f* m) { modelMatrix = m; }
//    void setCamera(const PerspectiveCamera* c) { cam = c; }
//    void setMainLight(const DirectionalLight* l) { mainLight = l; }
//    void setMaterial(const Material* m) { material = m; }
//    void setShadowMaterial(const Material* m) { shadowMaterial = m; }
//    void setConstantBuffer(ConstantBuffer* buffer) { materialConstantBuffer = buffer; }
//    void setShadowConstantBuffer(const ConstantBuffer* buffer) { shadowConstantBuffer = buffer; }
//
//private:
//    typedef void (*PR)(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord);
//    void renderBuffer(int width, int height, float* depthBuffer, uchar* colorBuffer, const Material* mat, const ConstantBuffer* cb, PR pixelRenderer);
//
//    int renderSingleTriangle(int width, int height, float* depthBuffer, uchar* colorBuffer, Triangle& tri, const Material* mat, const ConstantBuffer* cb, PR pixelRenderer);
//    static Vector4f screenMapping(int width, int height, Vector4f clipSpaceCoord);
//    static FragmentInput barycentricInterpolation(const FragmentInput* vertices, const Vector3f& perspectiveBaryCoord);
//
//    static void depthRenderer(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord);
//    static void shadowRenderer(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord);
//    static void colorRenderer(float* depthIndex, uchar* colorIndex, float depth, const Material* mat, const ConstantBuffer* cb, const FragmentInput* fi, const Vector3f& perspectiveBaryCoord);
//
//    const int screenWidth;
//    const int screenHeight;
//    const int shadowWidth;
//    bool enableEarlyZ;
//    bool enableShadow;
//
//    int pixelSize;
//    //Vector4f clearColor;
//
//	const tinyobj::ObjReader* reader = nullptr;
//	const Matrix4f* modelMatrix;
//
//    const PerspectiveCamera* cam;
//    const DirectionalLight* mainLight;
//    const Material* material;
//    ConstantBuffer* materialConstantBuffer;
//
//    std::vector<uchar> clearBuffer;
//
//    QPoint lastFrameMousePos;
//    clock_t lastFrameTime;
//    float deltaTime;
//
//    const ConstantBuffer* shadowConstantBuffer;
//    const Material* shadowMaterial;
//    const OrthographicCamera* shadowCam;
//};
//
//#endif // !RENDERER_H
