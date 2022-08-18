#include "SoftRasterizer.h"

SoftRasterizer::SoftRasterizer(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    resize(1000, 1000);

    img = QImage(imageWidth, imageHeight, QImage::Format_RGBA8888);
    data = img.bits();
    img.fill(Qt::white);

    label.setParent(this);
    label.setGeometry(0, 100, img.width(), img.height());
    label.show();
}

SoftRasterizer::~SoftRasterizer()
{
}

void SoftRasterizer::paintEvent(QPaintEvent*)
{
    clock_t start, end;

    start = clock();
    int pixIndex = 0;
	for (int i = 0; i < imageWidth; ++i) {
		for (int j = 0; j < imageHeight; ++j) {
			data[pixIndex + 0] = 128;
			data[pixIndex + 1] = 255;
			data[pixIndex + 2] = 255;
			data[pixIndex + 3] = 255;
			pixIndex += 4;
		}
	}
	end = clock();
	label.setPixmap(QPixmap::fromImage(img));

    qDebug() << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

    testEigen();
}

void SoftRasterizer::testEigen()
{
    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    qDebug() << m(0, 0) << m(1, 0) << m(0, 1) << m(1, 1) << endl;
}
