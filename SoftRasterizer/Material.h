#ifndef MATERIAL_H
#define MATERIAL_H

#include "Eigen/Core"
#include "Shader.h"

using namespace Eigen;

struct Material
{
	ShaderProperties shaderProps;
	VertexShader vertexShader;
	FragmentShader fragmentShader;
};


#endif // !MATERIAL_H
