#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"

const double DEFAULT_ACC = 0.03;

class TankPathSimulator :
	public BaseSimulator
{
public:
	TankPathSimulator(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time);
	void reset(double time);

	void getTankPosition(Vector tankPosition, Vector tankTangent);

protected:

	BaseSystem* m_object;
	Vector position, tangent;

	double vel, acc, lastTimeStamp;
};

