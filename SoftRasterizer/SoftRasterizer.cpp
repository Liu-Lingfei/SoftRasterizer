#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "SoftRasterizer.h"

SoftRasterizer::SoftRasterizer(QWidget* parent)
    :
    QMainWindow(parent),
    pixelSize(imageWidth* imageHeight),
    bufferSize(imageWidth* imageHeight * 4),
    clearBuffer(new uchar[bufferSize]),
    depthBuffer(new float[pixelSize]),
    img(imageWidth, imageHeight, QImage::Format_RGBA8888),
    frontBuffer(img.bits()),
    cam(Vector3f(0, 0, -10), Vector3f(0, 0, 0), Vector3f(0, 1, 0)),
    lastFrameMousePos(0, 0),
    deltaTime(0),
    M(Matrix4f::Identity()),
    MVP(Matrix4f::Identity())
{
    ui.setupUi(this);

    resize(1920, 1080);

    img.fill(Qt::white);

    label.setParent(this);
    label.setGeometry(0, 0, img.width(), img.height());
    label.show();

    //clearBuffer = new uchar[bufferSize];
    setClearColor(Vector4f(0, 0, 0, 255));

    //cam.setVFov(90);
    //cam.setRatio(float(imageWidth) / float(imageHeight));
    //cam.setNear(1);
    //cam.setFar(50);
    resetCamera();

    cam.updateTransform();
    cam.updateProjection();


    loadModel("");
}

SoftRasterizer::~SoftRasterizer()
{
    delete[] clearBuffer;
    delete[] depthBuffer;
}

void SoftRasterizer::drawTriangle(uchar* backBuffer, Triangle& tri, Vector4f color)
{
    clock_t start, end;
    Triangle rasterized;

    Vector3f a = tri.v[0].head(3);
    Vector3f b = tri.v[1].head(3);
    Vector3f c = tri.v[2].head(3);

    Vector3f crossNormal = -((b - a).cross(c - b)).normalized();
    Vector3f aveNormal = (tri.normals[0] + tri.normals[1] + tri.normals[2]).normalized();
    if (crossNormal.dot(aveNormal) < 0) {
        //qDebug() << "not counter clockwise";
        std::swap(tri.v[0], tri.v[2]);
        std::swap(tri.normals[0], tri.normals[2]);
        std::swap(tri.texCoords[0], tri.texCoords[2]);
        std::swap(tri.colors[0], tri.colors[2]);
    }

    rasterized.v[0] = screenMapping(MVP * tri.v[0]);
    rasterized.v[1] = screenMapping(MVP * tri.v[1]);
    rasterized.v[2] = screenMapping(MVP * tri.v[2]);

    Matrix3f model = M.topLeftCorner(3, 3);
    rasterized.normals[0] = model * tri.normals[0];
    rasterized.normals[1] = model * tri.normals[1];
    rasterized.normals[2] = model * tri.normals[2];

    rasterized.updateInfo();

    float minx = imageWidth, miny = imageHeight;
    float maxx = 0, maxy = 0;
    for (int i = 0; i < 3; i++) {
        minx = rasterized.v[i].x() < minx ? rasterized.v[i].x() : minx;
        maxx = rasterized.v[i].x() > maxx ? rasterized.v[i].x() : maxx;

        miny = rasterized.v[i].y() < miny ? rasterized.v[i].y() : miny;
        maxy = rasterized.v[i].y() > maxy ? rasterized.v[i].y() : maxy;
    }

    int iminx = floor(minx);
    int imaxx = ceil(maxx);
    int iminy = floor(miny);
    int imaxy = ceil(maxy);

    iminx = std::max(0, iminx);
    imaxx = std::min(imageWidth - 1, imaxx);
    iminy = std::max(0, iminy);
    imaxy = std::min(imageHeight - 1, imaxy);

    //qDebug("iminx = %d, imaxx = %d, iminy = %d, imaxy = %d", iminx, imaxx, iminy, imaxy);

    Vector3f bary(0, 0, 0);
    for (int x = iminx; x <= imaxx; x++) {
        for (int y = iminy; y <= imaxy; y++) {
            int pixelIndex = x + (imageHeight - 1 - y) * imageWidth;
            int startIndex = pixelIndex*4;
            if (rasterized.pointInsideTriangle(x+0.5, y+0.5, bary)) {
                float depth = bary.x() * rasterized.v[0].z() + bary.y() * rasterized.v[1].z() + bary.z() * rasterized.v[2].z();

                if (depth > 0 && depth > depthBuffer[pixelIndex]) {
                    depthBuffer[pixelIndex] = depth;
					Vector3f normal = bary.x() * rasterized.normals[0] + bary.y() * rasterized.normals[1] + bary.z() * rasterized.normals[2];
					Vector4f worldPos = bary.x() * tri.v[0] + bary.y() * tri.v[1] + bary.z() * tri.v[2];
					//backBuffer[startIndex + 0] = color.x();
					//backBuffer[startIndex + 1] = color.y();
					//backBuffer[startIndex + 2] = color.z();
					//backBuffer[startIndex + 3] = color.w();
					//qDebug("bary = (%f, %f, %f)", bary.x(), bary.y(), bary.z());

					//backBuffer[startIndex + 0] = bary.x()*256;
					//backBuffer[startIndex + 1] = bary.y()*256;
					//backBuffer[startIndex + 2] = bary.z()*256;
					//backBuffer[startIndex + 3] = 255;

					backBuffer[startIndex + 0] = worldPos.x()*100+100;
					backBuffer[startIndex + 1] = worldPos.y()*100+100;
					backBuffer[startIndex + 2] = worldPos.z()*100+100;
					backBuffer[startIndex + 3] = 255;

                    //backBuffer[startIndex + 0] = depth * 255;
                    //backBuffer[startIndex + 1] = depth * 255;
                    //backBuffer[startIndex + 2] = depth * 255;
                    //backBuffer[startIndex + 3] = 255;

                    //normal *= 0.5f;
                    //normal += Vector3f(0.5, 0.5, 0.5);
                    //normal *= 255;
                    //backBuffer[startIndex + 0] = normal.x();
                    //backBuffer[startIndex + 1] = normal.y();
                    //backBuffer[startIndex + 2] = normal.z();
                    //backBuffer[startIndex + 3] = 255;
                }
            }
        }
    }
}

void SoftRasterizer::setClearColor(const Vector4f& color)
{
    int pixIndex = 0;
    clearColor = color;
    for (int i = 0; i < imageWidth; ++i) {
        for (int j = 0; j < imageHeight; ++j) {
            clearBuffer[pixIndex + 0] = clearColor.x();
            clearBuffer[pixIndex + 1] = clearColor.y();
            clearBuffer[pixIndex + 2] = clearColor.z();
            clearBuffer[pixIndex + 3] = clearColor.w();
            pixIndex += 4;
        }
    }
}

void SoftRasterizer::clear(uchar* backBuffer)
{
    //memset(backBuffer, 255, bufferSize);
    std::fill(backBuffer, backBuffer + bufferSize, 255);
    //memcpy(backBuffer, clearBuffer, bufferSize);
    //std::fill(depthBuffer, depthBuffer + pixelSize, 0);
    memset(depthBuffer, 0, pixelSize * sizeof(float));
}

Vector4f SoftRasterizer::screenMapping(Vector4f clipSpaceCoord) const
{
    float invW = 1.0 / clipSpaceCoord.w();
    Vector4f NDC(
        clipSpaceCoord.x() * invW,
        clipSpaceCoord.y() * invW,
        clipSpaceCoord.z() * invW,
        invW
    );
    Vector4f screenSpaceCoord(
        (NDC.x()+1) * imageWidth * 0.5,
        (NDC.y()+1) * imageHeight * 0.5,
        NDC.z(),
        NDC.w()
    );
    return screenSpaceCoord;
}

void SoftRasterizer::loadModel(const std::string& filename)
{
    std::string inputfile = "./models/spot/spot_triangulated_good.obj";
    //std::string inputfile = "./models/bunny/bunny.obj";
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./models/spot/"; // Path to material files
    //reader_config.mtl_search_path = "./models/bunny/"; // Path to material files

    //tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(inputfile, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    //auto& attrib = reader.GetAttrib();
    //auto& shapes = reader.GetShapes();
    //auto& materials = reader.GetMaterials();

    //// Loop over shapes
    //for (size_t s = 0; s < shapes.size(); s++) {
    //    // Loop over faces(polygon)
    //    size_t index_offset = 0;
    //    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
    //        size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

    //        // Loop over vertices in the face.
    //        for (size_t v = 0; v < fv; v++) {
    //            // access to vertex
    //            tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
    //            tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
    //            tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
    //            tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
    //            //qDebug() << vx << ", " << vy << ", " << vz;

    //            // Check if `normal_index` is zero or positive. negative = no normal data
    //            if (idx.normal_index >= 0) {
    //                tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
    //                tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
    //                tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];
    //            }

    //            // Check if `texcoord_index` is zero or positive. negative = no texcoord data
    //            if (idx.texcoord_index >= 0) {
    //                tinyobj::real_t tx = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
    //                tinyobj::real_t ty = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];
    //            }

    //            // Optional: vertex colors
    //            // tinyobj::real_t red   = attrib.colors[3*size_t(idx.vertex_index)+0];
    //            // tinyobj::real_t green = attrib.colors[3*size_t(idx.vertex_index)+1];
    //            // tinyobj::real_t blue  = attrib.colors[3*size_t(idx.vertex_index)+2];
    //        }
    //        index_offset += fv;

    //        // per-face material
    //        shapes[s].mesh.material_ids[f];
    //    }
    //}
}

void SoftRasterizer::paintEvent(QPaintEvent*)
{
    //MyTransform t;
    //t.rotate(Vector3f(0, 1, 0), M_PI/4);
    //t.translate(Vector3f(100, 200, 123));
    //t.print();
    clock_t currTime = clock();
    deltaTime = float(currTime - lastFrameTime) / CLOCKS_PER_SEC;
    lastFrameTime = currTime;



    Matrix4f worldToCamera = cam.getWorldToCamera();
    Matrix4f projection = cam.getProjection();
    M = Matrix4f::Identity();
    MVP = projection * worldToCamera;

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();



    clock_t start, end;
    uchar* backBuffer = new uchar[bufferSize];
    memset(backBuffer, 0, bufferSize);

    start = clock();
    //for (int k = 0; k < 1000; ++k) {
	clear(backBuffer);


    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
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
                 tinyobj::real_t red   = attrib.colors[3*size_t(idx.vertex_index)+0];
                 tinyobj::real_t green = attrib.colors[3*size_t(idx.vertex_index)+1];
                 tinyobj::real_t blue  = attrib.colors[3*size_t(idx.vertex_index)+2];

                //qDebug() << red << ", " << green << ", " << blue;
                 tri.colors[v] = Vector4f(red, green, blue, 255);
            }
            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];

            drawTriangle(backBuffer, tri, Vector4f(255, 0, 0, 255));
        }
    }

	Vector4f a(-100, 0, 1, 1);
	Vector4f b(100, 0, 1, 1);
	Vector4f c(0, 100, 1, 1);

	Vector4f d(0, 0, 0, 1);
	Vector4f e(200, 0, 0, 1);
	Vector4f f(100, 100, 0, 1);

	Triangle tri2(d, e, f);
	Triangle tri(a, b, c);

	//drawTriangle(backBuffer, tri, Vector4f(255, 128, 128, 255));
	//drawTriangle(backBuffer, tri2, Vector4f(255, 128, 128, 255));

	memcpy(frontBuffer, backBuffer, bufferSize);
    //}

	end = clock();
	label.setPixmap(QPixmap::fromImage(img));
    delete[] backBuffer;
    qDebug() << CLOCKS_PER_SEC / double(end - start) << " FPS";
    qDebug() << double(end - start) / CLOCKS_PER_SEC  << " s";

    //testEigen();
}

void SoftRasterizer::keyPressEvent(QKeyEvent* e)
{
    const float speed = 5;
    
    float step = speed * deltaTime;

    Vector3f offset = Vector3f::Zero();
    Vector3f front = cam.z;
    Vector3f right = cam.x;
    Vector3f up = cam.y;
    switch (e->key())
    {
    case Qt::Key_W:
        offset = front;
        break;
    case Qt::Key_S:
        offset =  -front;
        break;
    case Qt::Key_A:
        offset =  -right;
        break;
    case Qt::Key_D:
        offset =  right;
        break;
    case Qt::Key_Q:
        offset =  -up;
        break;
    case Qt::Key_E:
        offset =  up;
        break;
    case Qt::Key_Space:
		resetCamera();
		break;
    }
    offset *= step;
    //qDebug("offset = (%f, %f, %f)", offset.x(), offset.y(), offset.z());
    //qDebug("oldPos = (%f, %f, %f)", cam.pos.x(), cam.pos.y(), cam.pos.z());
    cam.setPosition(cam.pos + offset);
    cam.setTarget(cam.target + offset);
    //qDebug("newPos = (%f, %f, %f)", cam.pos.x(), cam.pos.y(), cam.pos.z());
	cam.updateTransform();
}

void SoftRasterizer::mousePressEvent(QMouseEvent* e)
{
    lastFrameMousePos = e->globalPos();
}

void SoftRasterizer::mouseMoveEvent(QMouseEvent* e)
{
    QPoint currFrameMousePos = e->globalPos();
    QPoint offset = (currFrameMousePos - lastFrameMousePos);

    Vector2f rotation(float(offset.x()) / float(imageWidth), float(offset.y()) / float(imageHeight));
    float rotationRadian = rotation.norm() * M_PI;
    Vector2f rotationRadians = rotation * M_PI;

    Vector3f rotationAxis(offset.y(), offset.x(), 0);

    cam.rotate(rotationAxis, rotationRadian);
    //cam.rotateAroundY(rotationRadians.x());
    //cam.rotateAroundX(rotationRadians.y());
    cam.updateTransform();
    lastFrameMousePos = currFrameMousePos;
}

void SoftRasterizer::wheelEvent(QWheelEvent* e)
{
    const float speed = 0.01;
    float step = speed * e->angleDelta().y();
    float fov = cam.vfov;
    fov += step;
    fov = std::max(fov, 0.0f);
    fov = std::min(fov, 180.0f);
    qDebug() << "fov = " << fov;
    cam.setVFov(fov);
    cam.updateProjection();
}

void SoftRasterizer::testEigen()
{
    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    qDebug() << m(0, 0) << m(1, 0) << m(0, 1) << m(1, 1);
}

void SoftRasterizer::resetCamera()
{
    cam.setPosition(Vector3f(0, 0, -2));
    cam.setTarget(Vector3f(0, 0, 0));
    cam.setUp(Vector3f(0, 1, 0));

    cam.setVFov(90);
    cam.setRatio(float(imageWidth) / float(imageHeight));
    cam.setNear(0.1);
    cam.setFar(50);
}
