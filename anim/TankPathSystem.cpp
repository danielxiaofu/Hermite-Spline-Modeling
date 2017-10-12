#include "TankPathSystem.h"
#include <fstream>
#include <stdlib.h>
#include "GlobalResourceManager.h"


TankPathSystem::TankPathSystem(const std::string & name) :
	BaseSystem(name)
{
	zeroVector(m_pos);
}

void TankPathSystem::getState(double * p)
{
	p[0] = arcLength;
}

void TankPathSystem::setState(double * p)
{
	arcLength = p[0];

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
		
		//hermite->generateUniformCurve(uniformedHermite);
		hermite->visible = true;
		//uniformedHermite->visible = true;
		hermite->generateLengthTable();

		glutPostRedisplay();
		

	}
	else if (strcmp(argv[0], "tFromLength") == 0)
	{
		if (argc == 2)
		{
			Vector p, t;
			double l = atof(argv[1]);
			hermite->getPointFromLength(p, t, l);
			//animTcl::OutputMessage("t from length is %f, length = %f", result, length);

		}
	}

	return 0;
}

void TankPathSystem::reset(double time)
{

}

void TankPathSystem::setPath(Hermite * path)
{
	hermite = path;
}

void TankPathSystem::setUniformedPath(Hermite * uniformedPath)
{
	uniformedHermite = uniformedPath;
}

void TankPathSystem::display(GLenum mode)
{
	animTcl::OutputMessage("arcLength = %f", arcLength);
	hermite->getPointFromLength(m_pos, m_tangent, arcLength);

	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(m_pos[0], m_pos[1], m_pos[2]);
	glScalef(0.01, 0.01, 0.01);

	if (m_model.numvertices > 0)
		glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	else
		glutSolidSphere(1.0, 20, 20);

	glPopMatrix();
	glPopAttrib();
}

void TankPathSystem::loadModel()
{
	m_model.ReadOBJ("data/porsche.obj");
	glmFacetNormals(&m_model);
	glmVertexNormals(&m_model, 90);
}



