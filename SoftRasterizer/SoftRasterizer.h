#pragma once

//#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include <QtWidgets/QMainWindow>
#include "ui_SoftRasterizer.h"

#include "QPainter"
#include "QImage"
#include "QPixmap"
#include "QDebug"
#include "QLabel"
#include "QKeyEvent"
#include "QMouseEvent"
#include "QWheelEvent"

#include <ctime>
#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>

#include "Eigen/Dense"

#include "MathDefines.h"
#include "Triangle.h"
#include "Transform.h"
#include "Camera.h"

#include "Input.h"
//#include "VertexShader.h"
//#include "FragmentShader.h"
#include "Light.h"
#include "Shader.h"
#include "Material.h"

#include "tiny_obj_loader.h"
#include "FastRenderer.h"


using Eigen::MatrixXd;

class SoftRasterizer : public QMainWindow
{
    Q_OBJECT

public:
    SoftRasterizer(QWidget *parent = nullptr);
    SoftRasterizer(const SoftRasterizer& r) = delete;
    ~SoftRasterizer();

    


protected:
    void paintEvent(QPaintEvent* e);
    void keyPressEvent(QKeyEvent* e);
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void wheelEvent(QWheelEvent* e);

private:
    int drawTriangle(uchar* backBuffer, Triangle& tri);
    int drawTriangleDepth(float* buffer, Triangle& tri);
    int drawTriangleShadow(float* buffer, Triangle& tri);
    void setClearColor(const Vector4f& color);
    void clear();
    Vector4f screenMapping(int screenWidth, int screenHeight, Vector4f clipSpaceCoord) const;
    void loadModel(const std::string& filename);

    void resetCamera();
    void resetModelMatrix();
    void resetShadowCamera();
    void updateConstantBuffer();
    void updateShadowConstantBuffer();
    template<typename T>
    inline T interpolate(const Vector3f& baryCoord, const T& a, const T& b, const T& c);
    FragmentInput barycentricInterpolation(const FragmentInput* vertices, Vector3f& baryCoord, Vector3f& perspectiveBaryCoord);

    void renderDepthMap();
    void renderShadowMap();
    void render();

    //std::unique_ptr<Renderer> r;
    FastRenderer* r;
    Ui::SoftRasterizerClass ui;

    const int imageWidth = 1920;
    const int imageHeight = 1080;

    //const int imageWidth = 400;
    //const int imageHeight = 400;

    const int shadowWidth = 1024;

    QPixmap pix;
    QImage img;
    QLabel label;

    int pixelSize;
    int bufferSize;

    uchar* frontBuffer;

    //uchar* clearBuffer;
    //std::unique_ptr<uchar[]> clearBuffer;
    std::vector<uchar> clearBuffer;

    //float* depthBuffer;
    //std::unique_ptr<float[]> depthBuffer;
    bool earlyZ = false;
    std::vector<float> depthBuffer;

    std::vector<std::vector<float>> shadowMaps;

    //std::vector<

    Vector4f clearColor;

    Camera cam;

    QPoint lastFrameMousePos;
    clock_t lastFrameTime;
    float deltaTime;

    Matrix4f modelMatrix;
    //Matrix4f V;
    //Matrix4f P;
    //Matrix4f MVP;

    tinyobj::ObjReader reader;

    DirectionalLight mainLight;
    ConstantBuffer cb;
    Material m;

    //VertexShader vertexShader;
    //FragmentShader fragmentShader;

    ConstantBuffer shadowCb;
    Material shadowMaterial;
    DirectionalLightShadowCamera shadowCam;
};

template<typename T>
inline T SoftRasterizer::interpolate(const Vector3f& baryCoord, const T& a, const T& b, const T& c)
{
    return baryCoord.x() * a + baryCoord.y() * b + baryCoord.z() * c;
}
