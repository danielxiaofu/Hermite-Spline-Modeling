#include "MissileSystem.h"


MissileSystem::MissileSystem(const std::string & name) :
	BaseSystem(name)
{
	zeroVector(y0);
	zeroVector(y1);
	zeroVector(s0);
	zeroVector(s1);
	arcLength = 0.0;
	speed = DEFAULT_SPEED;
	updateData();
}

void MissileSystem::getState(double * p)
{
	updateData();

	for (int i = 0; i < 8; i++)
	{
		p[i] = data[i];
	}
}

void MissileSystem::setState(double * p)
{
	for (int i = 0; i < 8; i++)
	{
		data[i] = p[i];
	}
	path->visible = true;
}

int MissileSystem::command(int argc, myCONST_SPEC char ** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name);
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "state") == 0)
	{
		if (argc == 8)
		{
			double x = atof(argv[1]);
			double y = atof(argv[2]);
			double z = atof(argv[3]);
			double v1 = atof(argv[4]);
			double v2 = atof(argv[5]);
			double v3 = atof(argv[6]);
			double e = atof(argv[7]);
			Vector position;
			setVector(position, x, y, z);
			Quaternion quat = Quaternion(v1, v2, v3, e);
			double rotMatrix[3][3];
			quat.toMatrix(rotMatrix);
			rotatePoint_mat(s0, rotMatrix);
			VecNormalize(s0);
			VecScale(s0, 10.0);
			setStartPoint(position, s0);
			glutPostRedisplay();
		}
		else
		{
			animTcl::OutputMessage("system %s: wrong number of params after state.", m_name);
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "speed") == 0)
	{
		if (argc == 2)
			speed = atof(argv[1]);
		else
		{
			animTcl::OutputMessage("system %s: wrong number of params after speed.", m_name);
			return TCL_ERROR;
		}
	}
	return 0;
}

void MissileSystem::reset(double time)
{
	zeroVector(y0);
	zeroVector(y1);
	zeroVector(s0);
	zeroVector(s1);
	arcLength = 0.0;
	speed = DEFAULT_SPEED;
	updateData();
	initializePath();

}

void MissileSystem::setPath(Hermite * missilePath)
{
	path = missilePath;
}

void MissileSystem::setTargetPoint(Vector position, Vector tangent)
{
	VecCopy(y1, position);
	VecCopy(s1, tangent);
	path->setPoint(1, y1[0], y1[1], y1[2]);
	path->setTangent(1, s1[0], s1[1], s1[2]);
	path->generateLengthTable();
}

void MissileSystem::setStartPoint(Vector position, Vector tangent)
{
	VecCopy(y0, position);
	VecCopy(s0, tangent);
	path->setPoint(0, y0[0], y0[1], y0[2]);
	path->setTangent(0, s0[0], s0[1], s0[2]);
	path->generateLengthTable();
}

void MissileSystem::display(GLenum mode)
{
	// re-compute path
	Vector newPos, newTan;
	setVector(newPos, data[1], data[2], data[3]);
	setVector(newTan, data[4], data[5], data[6]);
	VecNormalize(newTan);
	VecScale(newTan, 20);
	setTargetPoint(newPos, newTan);
	arcLength = data[0];
	//animTcl::OutputMessage("arclength = %f", arcLength);
	path->getPointFromLength(newPos, newTan, arcLength);

	// calculate rotation
	double angle = 0.0;
	double yAxis[3] = { 0.0, -1.0, 0.0 };
	Vector axis, normalizedTangent;
	VecCopy(normalizedTangent, newTan);
	VecNormalize(normalizedTangent);
	quaternion.rotateAxis(yAxis, normalizedTangent);
	quaternion.getAxisAngle(axis, &angle);
	angle = angle * 180.0 / M_PI;

	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(newPos[0], newPos[1], newPos[2]);
	glRotated(90.0, 1.0, 0.0, 0.0);
	glRotated(angle, axis[0], axis[2], axis[1]);

	glScalef(0.7, 0.7, 0.7);

	if (m_model.numvertices > 0)
		glmDraw(&m_model, GLM_SMOOTH);
	else
		glutSolidSphere(1.0, 20, 20);

	glPopMatrix();
	glPopAttrib();
}

void MissileSystem::loadModel()
{
	m_model.ReadOBJ("data/missile.obj");
	glmFacetNormals(&m_model);
	glmVertexNormals(&m_model, 90);
}

void MissileSystem::initializePath()
{
	path->reset(0.0);
	setVector(s0, rand() % 10 - 5.0, rand() % 10 - 5.0, 0.0);
	VecNormalize(s0);
	VecScale(s0, 10);
	setVector(y0, rand() % 10 - 5.0, rand() % 10 - 5.0, 0.0);
	//setVector(y0, 5, 5, 0.0);
	path->addControlPoint(y0[0], y0[1], y0[2], s0[0], s0[1], s0[2]);
	path->addControlPoint(y1[0], y1[1], y1[2], s1[0], s1[1], s1[2]);
	path->visible = false;
}

void MissileSystem::updateData()
{
	data[0] = arcLength;
	data[1] = y1[0];
	data[2] = y1[1];
	data[3] = y1[2];
	data[4] = s1[0];
	data[5] = s1[1];
	data[6] = s1[2];
	data[7] = speed;
}