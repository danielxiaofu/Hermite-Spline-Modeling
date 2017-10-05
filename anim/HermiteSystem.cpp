#include "HermiteSystem.h"
#include <fstream>
#include <stdlib.h>
#include "GlobalResourceManager.h"

HermiteSystem::HermiteSystem(const std::string & name) :
	BaseSystem(name)
{

}

void HermiteSystem::getState(double * p)
{
}

void HermiteSystem::setState(double * p)
{
}

int HermiteSystem::command(int argc, myCONST_SPEC char ** argv)
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

		hermite = dynamic_cast<Hermite*>(GlobalResourceManager::use()->getObject("Hermite1"));
		assert(hermite);

		double px, py, pz, sx, sy, sz;
		int i = 0;
		while (myFile >> px >> py >> pz >> sx >> sy >> sz && i < numPoints)
		{
			//animTcl::OutputMessage("%f, %f, %f, %f, %f, %f", px, py, pz, sx, sy, sz);
			hermite->addControlPoint(px, py, pz, sx, sy, sz);
		}
		//hermite->crInitialize();
		glutPostRedisplay();
		hermite->generateLengthTable();

	}
	else if (strcmp(argv[0], "set") == 0)
	{
		if (strcmp(argv[1], "point") == 0)
		{
			if (argc == 6)
			{
				int index = atoi(argv[2]);
				double x = atof(argv[3]);
				double y = atof(argv[4]);
				double z = atof(argv[5]);
				if(hermite->setPoint(index, x, y, z))
					glutPostRedisplay();
				else
					animTcl::OutputMessage("system %s: set point failed, make sure the index is valid", m_name);
			}
			else 
			{
				animTcl::OutputMessage("system %s: wrong number of params after set point.", m_name);
				return TCL_ERROR;
			}
		}
		else if (strcmp(argv[1], "tangent") == 0)
		{
			if (argc == 6)
			{
				int index = atoi(argv[2]);
				double x = atof(argv[3]);
				double y = atof(argv[4]);
				double z = atof(argv[5]);
				if (hermite->setTangent(index, x, y, z))
					glutPostRedisplay();
				else
					animTcl::OutputMessage("system %s: set tangent failed, make sure the index is valid", m_name);
			}
			else
			{
				animTcl::OutputMessage("system %s: wrong number of params after set tangent.", m_name);
				return TCL_ERROR;
			}
		}
		
	}
	else if (strcmp(argv[0], "getArcLength") == 0)
	{
		if (argc == 2)
		{
			std::string t;
			
			double l = hermite->getArcLength(atof(argv[1]));
			//animTcl::OutputMessage("argv[1] = %f", atof(argv[1]));
			animTcl::OutputMessage("arc length = %f", l);
		}
		else
		{
			animTcl::OutputMessage("system %s: wrong number of params after getArcLength.", m_name);
			return TCL_ERROR;
		}
		
	}
	else if (strcmp(argv[0], "add") == 0)
	{
		if (strcmp(argv[1], "point") == 0)
		{
			if (argc == 8)
			{
				double x = atof(argv[2]);
				double y = atof(argv[3]);
				double z = atof(argv[4]);
				double sx = atof(argv[5]);
				double sy = atof(argv[6]);
				double sz = atof(argv[7]);
				hermite->addControlPoint(x, y, z, sx, sy, sz);
				glutPostRedisplay();
			}
			else
			{
				animTcl::OutputMessage("system %s: wrong number of params after add point.", m_name);
			}
			
		}
	}

	return 0;
}
