#include "TankPathSystem.h"
#include <fstream>
#include <stdlib.h>
#include "GlobalResourceManager.h"


TankPathSystem::TankPathSystem(const std::string & name) :
	BaseSystem(name)
{
	zeroVector(m_pos);
	zeroVector(m_axis);
	arcLength = 0.0;
	totalLength = 0.0;
	data[0] = arcLength;
	data[1] = totalLength;

}

void TankPathSystem::getState(double * p)
{
	updateData();
	for (int i = 0; i < 8; i++)
	{
		p[i] = data[i];
	}
}

void TankPathSystem::setState(double * p)
{
	for (int i = 0; i < 8; i++)
	{
		data[i] = p[i];
	}
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
		
		hermite->visible = true;
		hermite->generateLengthTable();

		glutPostRedisplay();
		totalLength = hermite->getArcLength(1.0); // get total length

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
	zeroVector(m_pos);
	zeroVector(m_axis);
	arcLength = 0.0;
	totalLength = 0.0;
	data[0] = arcLength;
	data[1] = totalLength;
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
	
	arcLength = data[0];
	//animTcl::OutputMessage("arcLength = %f", arcLength);
	hermite->getPointFromLength(m_pos, m_tangent, arcLength);

	// calculate rotation
	double angle = 0.0;
	double yAxis[3] = { 0.0, -1.0, 0.0 };
	Vector axis, normalizedTangent;
	VecCopy(normalizedTangent, m_tangent);
	VecNormalize(normalizedTangent);
	quaternion.rotateAxis(yAxis, normalizedTangent);
	quaternion.getAxisAngle(axis, &angle);
	angle = angle * 180.0 / M_PI;

	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(m_pos[0], m_pos[1], m_pos[2]);
	glRotated(90.0, 1.0, 0.0, 0.0);
	glRotated(angle, axis[0], axis[2], axis[1]);
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

void TankPathSystem::getPosition(Vector position, Vector tangent)
{
	VecCopy(position, m_pos);
	VecCopy(tangent, m_tangent);
}

void TankPathSystem::updateData()
{
	data[0] = arcLength;
	data[1] = totalLength;
	data[2] = m_pos[0];
	data[3] = m_pos[1];
	data[4] = m_pos[2];
	data[5] = m_tangent[0];
	data[6] = m_tangent[1];
	data[7] = m_tangent[2];
}



