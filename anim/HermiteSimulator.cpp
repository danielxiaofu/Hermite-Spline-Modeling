#include "HermiteSimulator.h"

HermiteSimulator::HermiteSimulator(const std::string & name, BaseSystem * target):
	BaseSimulator(name),
	m_object(target)
{
}

int HermiteSimulator::step(double time)
{
	return 0;
}

int HermiteSimulator::init(double time)
{
	return 0;
}


