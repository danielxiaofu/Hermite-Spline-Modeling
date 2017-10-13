#include "TankPathSimulator.h"
#include "GlobalResourceManager.h"
#include "TankPathSystem.h"

TankPathSimulator::TankPathSimulator(const std::string & name, BaseSystem * target):
	BaseSimulator(name),
	m_object(target)
{
	zeroVector(position);
	zeroVector(tangent);
	acc = DEFAULT_ACC;
	vel = 0.0;
	lastTimeStamp = 0.0;
}

int TankPathSimulator::step(double time)
{
	double actualTime = time;
	double deltaTime = actualTime - lastTimeStamp;
	lastTimeStamp = actualTime;

	double data[8];
	m_object->getState(data);
	double easyInStop = data[1] * 0.1;
	double easyOutStop = data[1] * 0.9;

	if (data[0] < easyInStop)
		vel += acc;
	else if (data[0] > easyOutStop)
		vel -= acc;
	else if (fabs(data[0] - data[1]) < DBL_EPSILON)
	{
		acc = 0;
		vel = 0;
	}
	if (vel < 0) // prevent from reversing
		vel = 0;

	data[0] += (vel * deltaTime);
	setVector(position, data[2], data[3], data[4]);
	setVector(tangent, data[5], data[6], data[7]);

	m_object->setState(data);

	return 0;
}

int TankPathSimulator::init(double time)
{
	return 0;
}

void TankPathSimulator::getTankPosition(Vector tankPosition, Vector tankTangent)
{
	VecCopy(tankPosition, position);
	VecCopy(tankTangent, tangent);
}

void TankPathSimulator::reset(double time)
{
	zeroVector(position);
	zeroVector(tangent);
	acc = DEFAULT_ACC;
	vel = 0.0;
	lastTimeStamp = 0.0;
}
