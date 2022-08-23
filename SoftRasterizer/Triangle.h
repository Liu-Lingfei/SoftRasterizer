#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "Eigen/Core"
#include "qdebug.h"

using namespace Eigen;

class Triangle {
public:
	Triangle();
	Triangle(const Vector4f& a, const Vector4f& b, const Vector4f c);

	void setVertex(int ind, Vector4f ver);
	void setNormal(int ind, Vector3f n);
	void setColor(int ind, float r, float g, float b, float a);

	void setNormals(const std::array<Vector3f, 3>& normals);
	void setColors(const std::array<Vector4f, 3>& colors);
	//void setTexCoord(int ind, Vector2f uv);
	Vector3f precomputeLineEquations(float x, float y) const;
	bool pointInsideTriangleFast(float x, float y, const Vector3f& r, Vector3f& baryCoord) const;
	bool pointInsideTriangle(float x, float y, Vector3f& baryCoord, Vector3f& pespectiveBary) const;
	void updateInfo();

	//w is reversed after vertex shader
	Vector4f v[3];
	Vector4f colors[3];
	Vector2f texCoords[3];
	Vector3f normals[3];
	bool isTopEdge[3];
	bool isLeftEdge[3];

	//e_i(x, y) = a_i*x + b_i*y + c_i;
	//edgeCoeff[i] = Vector3f(a_i, b_i, c_i);
	Vector3f edgeCoeffs[3];

private:
	void computeCoeffs();
	float computeLineEquation(const Vector3f& p, int index) const;
	//inline bool isTopEdge(int index) const;
	//inline bool isLeftEdge(int index) const;
};


#endif // !TRIANGLE_H
