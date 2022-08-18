#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_SoftRasterizer.h"

class SoftRasterizer : public QMainWindow
{
    Q_OBJECT

public:
    SoftRasterizer(QWidget *parent = nullptr);
    ~SoftRasterizer();

private:
    Ui::SoftRasterizerClass ui;
};
