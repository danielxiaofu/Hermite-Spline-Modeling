#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"

class HermiteSimulator :
	public BaseSimulator
{
public:

	HermiteSimulator(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time);

protected:

	BaseSystem* m_object;
};

