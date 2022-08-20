#include "Triangle.h"

Triangle::Triangle()
{
    v[0] << 0, 0, 0, 1;
    v[1] << 0, 0, 0, 1;
    v[2] << 0, 0, 0, 1;

    colors[0] << 0, 0, 0, 255;
    colors[1] << 0, 0, 0, 255;
    colors[2] << 0, 0, 0, 255;

    texCoords[0] << 0.0, 0.0;
    texCoords[1] << 0.0, 0.0;
    texCoords[2] << 0.0, 0.0;
}

Triangle::Triangle(const Vector4f& a, const Vector4f& b, const Vector4f c)
{
    v[0] = a;
    v[1] = b;
    v[2] = c;

    colors[0] << 0, 0, 0, 255;
    colors[1] << 0, 0, 0, 255;
    colors[2] << 0, 0, 0, 255;

    texCoords[0] << 0.0, 0.0;
    texCoords[1] << 0.0, 0.0;
    texCoords[2] << 0.0, 0.0;
}

inline void Triangle::setVertex(int ind, Vector4f ver)
{
    v[ind] = ver;
}

inline void Triangle::setNormal(int ind, Vector3f n)
{
    normals[ind] = n;
}

inline void Triangle::setColor(int ind, float r, float g, float b, float a)
{
    colors[ind](0) = r;
    colors[ind](1) = g;
    colors[ind](2) = b;
    colors[ind](3) = a;
}

void Triangle::setNormals(const std::array<Vector3f, 3>& normals)
{
    this->normals[0] = normals[0];
    this->normals[1] = normals[1];
    this->normals[2] = normals[2];
}

void Triangle::setColors(const std::array<Vector4f, 3>& colors)
{
    this->colors[0] = colors[0];
    this->colors[1] = colors[1];
    this->colors[2] = colors[2];
}

Vector3f Triangle::precomputeLineEquations(float x, float y) const
{
    Vector3f p(x, y, 1);
    float r0 = computeLineEquation(p, 0);
    float r1 = computeLineEquation(p, 1);
    float r2 = computeLineEquation(p, 2);

    //qDebug("r0 = %f, r1 = %f, r2 = %f", r0, r1, r2);
    return Vector3f(r0, r1, r2);
}

bool Triangle::pointInsideTriangleFast(float x, float y, const Vector3f& r, Vector3f& baryCoord) const
{
    Vector3f p(x, y, 1);
    float r0 = r.x();
    float r1 = r.y();
    float r2 = r.z();

    //qDebug("r0 = %f, r1 = %f, r2 = %f", r0, r1, r2);

    float e0 = r0 * v[0].w();
    float e1 = r1 * v[1].w();
    float e2 = r2 * v[2].w();
    float sum = e0 + e1 + e2;
    float reversedSum = 1.0 / sum;
    baryCoord(0) = e0 * reversedSum;
    baryCoord(1) = e1 * reversedSum;
    baryCoord(2) = e2 * reversedSum;

    if (r0 < 0 || r1 < 0 || r2 < 0) return false;
    if (r0 > 0 && r1 > 0 && r2 > 0) return true;

    if (r0 == 0 && (isTopEdge[0] || isLeftEdge[0])) return true;
    if (r1 == 0 && (isTopEdge[1] || isLeftEdge[1])) return true;
    if (r2 == 0 && (isTopEdge[2] || isLeftEdge[2])) return true;
}

void Triangle::updateInfo()
{
    computeCoeffs();
}

void Triangle::computeCoeffs()
{
    Vector2f p01 = v[1].head(2) - v[0].head(2);
    edgeCoeffs[2](0) = -p01.y();
    edgeCoeffs[2](1) = p01.x();
    edgeCoeffs[2](2) = p01.y() * v[0].x() - p01.x() * v[0].y();

    Vector2f p12 = v[2].head(2) - v[1].head(2);
    edgeCoeffs[0](0) = -p12.y();
    edgeCoeffs[0](1) = p12.x();
    edgeCoeffs[0](2) = p12.y() * v[1].x() - p12.x() * v[1].y();

    Vector2f p20 = v[0].head(2) - v[2].head(2);
    edgeCoeffs[1](0) = -p20.y();
    edgeCoeffs[1](1) = p20.x();
    edgeCoeffs[1](2) = p20.y() * v[2].x() - p20.x() * v[2].y();

    //qDebug("coeff[0] = %f %f, %f", edgeCoeffs[0].x(), edgeCoeffs[0].y(), edgeCoeffs[0].z());
    //qDebug("coeff[1] = %f %f, %f", edgeCoeffs[1].x(), edgeCoeffs[1].y(), edgeCoeffs[1].z());
    //qDebug("coeff[2] = %f %f, %f", edgeCoeffs[2].x(), edgeCoeffs[2].y(), edgeCoeffs[2].z());

    for (int i = 0; i < 3; ++i) {
		isTopEdge[i] = edgeCoeffs[i].x() == 0 && edgeCoeffs[i].y() < 0;
		isLeftEdge[i] = edgeCoeffs[i].x() > 0;
    }
}

inline float Triangle::computeLineEquation(const Vector3f& p, int index) const
{
    return edgeCoeffs[index].dot(p);
}
