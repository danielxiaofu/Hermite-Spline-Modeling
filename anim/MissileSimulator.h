#pragma once
#include "BaseSimulator.h"
#include "TankPathSimulator.h"
#include "BaseSystem.h"

class MissileSimulator :
	public BaseSimulator
{
public:
	MissileSimulator(const std::string& name, BaseSystem* target);
	
	int step(double time);
	int init(double time);
	void reset(double time);


	void setTargetTank(TankPathSimulator* target);

protected:

	BaseSystem* m_object;
	TankPathSimulator* targetSimulator; // a pointer to the tank that this missile will chase

	double lastTimeStamp;
};

