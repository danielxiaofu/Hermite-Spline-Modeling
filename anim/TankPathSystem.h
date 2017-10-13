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

	void getPosition(Vector position, Vector tangent);

protected:
	Hermite* hermite;
	Hermite* uniformedHermite;

	double arcLength, totalLength;
	Vector m_pos, m_tangent, m_axis;
	Quaternion quaternion;

	/* data of the tank path system, start from 0: 
	arcLengh, totalLength, 
	position.x, position.y, position.z, 
	tangent.x, tangent.y, tangent.z
	*/
	double data[8];

	GLMmodel m_model;

	void updateData();
};

