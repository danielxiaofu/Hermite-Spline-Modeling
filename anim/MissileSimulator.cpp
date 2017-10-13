#include "MissileSimulator.h"
#include "GlobalResourceManager.h"

MissileSimulator::MissileSimulator(const std::string & name, BaseSystem * target) :
	BaseSimulator(name),
	m_object(target)
{
	lastTimeStamp = 0.0;
}

int MissileSimulator::step(double time)
{
	double actualTime = time;
	double deltaTime = actualTime - lastTimeStamp;
	lastTimeStamp = actualTime;

	double data[8];
	m_object->getState(data);
	// get target position
	Vector pos, tan;
	if (targetSimulator)
	{
		targetSimulator->getTankPosition(pos, tan);
	}
	data[0] += data[7] * deltaTime;
	//animTcl::OutputMessage("time = %f", time);
	data[1] = pos[0];
	data[2] = pos[1];
	data[3] = pos[2];
	data[4] = tan[0];
	data[5] = tan[1];
	data[6] = tan[2];
	m_object->setState(data);

	return 0;
}

int MissileSimulator::init(double time)
{
	return 0;
}

void MissileSimulator::reset(double time)
{
	lastTimeStamp = 0.0;
}

void MissileSimulator::setTargetTank(TankPathSimulator * target)
{
	targetSimulator = target;
}


