#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"

class TankPathSimulator :
	public BaseSimulator
{
public:
	TankPathSimulator(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time);

protected:

	BaseSystem* m_object;

	double lastTimeStamp = 0.0f;
};

