#ifndef LIGHT_H
#define LIGHT_H

#include "Eigen/Core"
using namespace Eigen;

struct DirectionalLight
{
	Vector3f position;
	Vector3f direction;
	Vector3f color;
	float shadowAttenuation = 1.0f;
	float distanceAttenuation = 1.0f;
};

#endif // !LIGHT_H
