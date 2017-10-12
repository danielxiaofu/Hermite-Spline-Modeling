#include "TankPathSimulator.h"
#include "GlobalResourceManager.h"

TankPathSimulator::TankPathSimulator(const std::string & name, BaseSystem * target):
	BaseSimulator(name),
	m_object(target)
{

}

int TankPathSimulator::step(double time)
{
	double actualTime = GlobalResourceManager::use()->getActualTime();
	double deltaTime = actualTime - lastTimeStamp;
	lastTimeStamp = actualTime;

	double vel = 4; // arclength/sec
	
	double arcLength;
	m_object->getState(&arcLength);
	arcLength += (vel * deltaTime);
	m_object->setState(&arcLength);

	return 0;
}

int TankPathSimulator::init(double time)
{
	return 0;
}
