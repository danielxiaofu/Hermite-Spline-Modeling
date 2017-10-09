#pragma once
#include "BaseSystem.h"
#include "Hermite.h"

class HermiteSystem :
	public BaseSystem
{
public:

	HermiteSystem(const std::string& name);

	void getState(double *p);

	void setState(double *p);

	int command(int argc, myCONST_SPEC char **argv);

	void onLeftClick(Vector position);

	void setHermiteObject(Hermite* hermiteP);

protected:
	Hermite* hermite;

};

