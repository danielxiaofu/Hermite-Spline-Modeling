#pragma once
#include "BaseSystem.h"
#include "Hermite.h"

class TankPathSystem : public BaseSystem
{

public:
	TankPathSystem(const std::string& name);
	
	void getState(double *p);

	void setState(double *p);

	int command(int argc, myCONST_SPEC char **argv);

	void reset(double time);

	void setPath(Hermite* path);

	void setUniformedPath(Hermite* uniformedPath);

	void display(GLenum mode = GL_RENDER);

	void loadModel();

protected:
	Hermite* hermite;
	Hermite* uniformedHermite;

	double arcLength = 0.0;
	Vector m_pos, m_tangent;

	GLMmodel m_model;
};

