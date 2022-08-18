#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_SoftRasterizer.h"

#include "qpainter.h"
#include "qimage.h"
#include "qpixmap.h"
#include "qdebug.h"
#include "qlabel.h"

#include <ctime>
#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include "Eigen/Dense"
using Eigen::MatrixXd;

class SoftRasterizer : public QMainWindow
{
    Q_OBJECT

public:
    SoftRasterizer(QWidget *parent = nullptr);
    ~SoftRasterizer();

protected:
    void paintEvent(QPaintEvent*);

private:
    void testEigen();

    Ui::SoftRasterizerClass ui;

    const int imageWidth = 1000;
    const int imageHeight = 800;

    QPixmap pix;
    QImage img;
    QLabel label;
    uchar* data;
};
