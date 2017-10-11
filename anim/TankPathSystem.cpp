#include "TankPathSystem.h"
#include <fstream>
#include <stdlib.h>
#include "GlobalResourceManager.h"


TankPathSystem::TankPathSystem(const std::string & name) :
	BaseSystem(name)
{

}

void TankPathSystem::getState(double * p)
{

}

void TankPathSystem::setState(double * p)
{

}

int TankPathSystem::command(int argc, myCONST_SPEC char ** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name);
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "load") == 0)
	{
		if (argc < 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params after load.", m_name);
			return TCL_ERROR;
		}

		std::ifstream myFile;
		myFile.open(argv[1]);
		if (!myFile)
		{
			animTcl::OutputMessage("system %s: unable to open file", m_name);
			return TCL_ERROR;
		}

		std::string fileName;
		int numPoints;
		myFile >> fileName >> numPoints;

		double px, py, pz, sx, sy, sz;
		int i = 0;
		while (myFile >> px >> py >> pz >> sx >> sy >> sz && i < numPoints)
		{
			//animTcl::OutputMessage("%f, %f, %f, %f, %f, %f", px, py, pz, sx, sy, sz);
			hermite->addControlPoint(px, py, pz, sx, sy, sz);
		}
		myFile.close();
		//hermite->crInitialize();
		glutPostRedisplay();
		hermite->generateLengthTable();

	}

	return 0;
}



