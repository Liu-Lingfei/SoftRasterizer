#include "SoftRasterizer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SoftRasterizer w;
    w.show();
    return a.exec();
}
