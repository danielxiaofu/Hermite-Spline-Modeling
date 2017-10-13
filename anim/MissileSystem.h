#pragma once
#include "BaseSystem.h"
#include "Hermite.h"

const double DEFAULT_SPEED = 3.0;

class MissileSystem :
	public BaseSystem
{
public:
	MissileSystem(const std::string& name);
	
	void getState(double *p);

	void setState(double *p);

	int command(int argc, myCONST_SPEC char **argv);

	void reset(double time);

	void setPath(Hermite* missilePath);

	void setTargetPoint(Vector position, Vector tangent);

	void setStartPoint(Vector position, Vector tangent);

	void display(GLenum mode = GL_RENDER);

	void loadModel();

	void initializePath();

protected:
	double arcLength, speed;
	Hermite* path;

	Vector y0, y1, s0, s1;
	Quaternion quaternion;

	GLMmodel m_model;

	/* data of missle system, start from 0: arcLength, targetPosition.x, targetPosition.y, targetPosition.z,
	targetTangent.x, targetTangent.y, targetTangent.z, speed
	*/
	double data[8];

	void updateData();
};

