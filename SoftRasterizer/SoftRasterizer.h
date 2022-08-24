#ifndef SOFT_RASTERIZER
#define SOFT_RASTERIZER



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
#include <chrono>

#include "Eigen/Dense"
using Eigen::MatrixXd;

#include "MathDefines.h"

#include "Scene.h"

#include <QtConcurrent/QtConcurrent>
using namespace QtConcurrent;

#include <thread>



class SoftRasterizer : public QMainWindow
{
    Q_OBJECT

public:
    SoftRasterizer(QWidget *parent = nullptr);
    //SoftRasterizer(const SoftRasterizer& r) = delete;
    ~SoftRasterizer();
    


protected:
    void paintEvent(QPaintEvent* e);
    void keyPressEvent(QKeyEvent* e);
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void wheelEvent(QWheelEvent* e);

private:
    const int imageWidth = 1024;
    const int imageHeight = 1024;
    const int shadowWidth = 512;

    Ui::SoftRasterizerClass ui;
    QLabel* fpsLabel;
    QLabel* resolutionLabel;
    QLabel* shadowWidthLabel;


    QPixmap pix;
    QImage *img;
    uchar* frontBuffer;
    QImage *depthImg;
    uchar* depthBuffer;

    Scene scene;
    
    QPoint lastFrameMousePos;
    clock_t lastFrameTime;
    float deltaTime;

    int frameCount;
    int frameRate;
};

//template<typename T>
//inline T SoftRasterizer::interpolate(const Vector3f& baryCoord, const T& a, const T& b, const T& c)
//{
//    return baryCoord.x() * a + baryCoord.y() * b + baryCoord.z() * c;
//}

#endif // !SOFT_RASTERIZER
