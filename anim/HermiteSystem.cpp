#include "HermiteSystem.h"
#include <fstream>
#include <stdlib.h>
#include "GlobalResourceManager.h"

HermiteSystem::HermiteSystem(const std::string & name) :
	BaseSystem(name)
{
	crMode = false;
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
		hermite->reset(0.0);

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
				if (hermite->setPoint(index, x, y, z))
				{
					glutPostRedisplay();
					hermite->generateLengthTable();
				}
					
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
				{
					glutPostRedisplay();
					hermite->generateLengthTable();
				}
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
				hermite->generateLengthTable();
				glutPostRedisplay();
			}
			else
			{
				animTcl::OutputMessage("system %s: wrong number of params after add point.", m_name);
			}
			
		}
	}
	else if (strcmp(argv[0], "cr") == 0)
	{
		if (!crMode)
		{
			hermite->applyCR();
			hermite->generateLengthTable();
			crMode = true;
		}
		else
		{
			hermite->turnOffCR();
			hermite->generateLengthTable();
			crMode = false;
		}
		glutPostRedisplay();
	}
	else if (strcmp(argv[0], "export") == 0)
	{
		if (argc < 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params after export.", m_name);
			return TCL_ERROR;
		}

		std::ofstream out(argv[1]);
		int n = hermite->getNumPoints();

		out << "hermite " << n << "\n";
		Vector point, tangent;
		for (int i = 0; i < n; i++)
		{
			hermite->getControlPoint(point, i);
			hermite->getControlPointTangent(tangent, i);
			out << point[0] << " " << point[1] << " " << point[2] << " " << tangent[0] << " " << tangent[1] << " " << tangent[2] << "\n";
		}
		out.close();
		animTcl::OutputMessage("system %s: export finished!.", m_name);
	}
	return 0;
}

void HermiteSystem::onLeftClick(Vector position)
{
	// zero out z value
	position[2] = 0;
	hermite->addControlPoint(position[0], position[1], position[2]);
	glutPostRedisplay();
}

void HermiteSystem::setHermiteObject(Hermite * hermiteP)
{
	hermite = hermiteP;
}
