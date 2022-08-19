#pragma once

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

#include "tiny_obj_loader.h"


using Eigen::MatrixXd;

class SoftRasterizer : public QMainWindow
{
    Q_OBJECT

public:
    SoftRasterizer(QWidget *parent = nullptr);
    SoftRasterizer(const SoftRasterizer& r) = delete;
    ~SoftRasterizer();

    void drawTriangle(uchar* backBuffer, Triangle& tri, Vector4f color);
    void setClearColor(const Vector4f& color);
    void clear(uchar* backBuffer);
    Vector4f screenMapping(Vector4f clipSpaceCoord) const;
    void loadModel(const std::string& filename);


protected:
    void paintEvent(QPaintEvent* e);
    void keyPressEvent(QKeyEvent* e);
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void wheelEvent(QWheelEvent* e);

private:
    void testEigen();
    void resetCamera();

    Ui::SoftRasterizerClass ui;

    const int imageWidth = 1920;
    const int imageHeight = 1080;

    QPixmap pix;
    QImage img;
    QLabel label;

    int pixelSize;
    int bufferSize;
    uchar* frontBuffer;
    uchar* clearBuffer;
    float* depthBuffer;

    Vector4f clearColor;

    Camera cam;

    QPoint lastFrameMousePos;
    clock_t lastFrameTime;
    float deltaTime;

    Matrix4f M;
    Matrix4f MVP;

    tinyobj::ObjReader reader;
};
