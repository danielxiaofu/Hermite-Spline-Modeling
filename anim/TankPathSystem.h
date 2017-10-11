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

protected:
	Hermite* hermite;

};

