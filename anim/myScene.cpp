////////////////////////////////////////////////////
// // Template code for  CS 174C
// // Modified by Yixiao Yang
////////////////////////////////////////////////////

#ifdef WIN32
#include <windows.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <shared/defs.h>

#include "shared/opengl.h"

#include <string.h>
#include <util/util.h>
#include <GLModel/GLModel.h>
#include "anim.h"
#include "animTcl.h"
#include "myScene.h"
#include "SampleParticle.h"
#include "SampleGravitySimulator.h"
#include "HermiteSystem.h"
#include "HermiteSimulator.h"
#include "TankPathSystem.h"
#include "TankPathSimulator.h"
#include "MissileSystem.h"
#include "MissileSimulator.h"
#include <util/jama/tnt_stopwatch.h>
#include <util/jama/jama_lu.h>

// register a sample variable with the shell.
// Available types are:
// - TCL_LINK_INT 
// - TCL_LINK_FLOAT

int g_testVariable = 10;

SETVAR myScriptVariables[] = {
	"testVariable", TCL_LINK_INT, (char *) &g_testVariable,
	"",0,(char *) NULL
};


//---------------------------------------------------------------------------------
//			Hooks that are called at appropriate places within anim.cpp
//---------------------------------------------------------------------------------

// start or end interaction
void myMouse(int button, int state, int x, int y)
{

	// let the global resource manager know about the new state of the mouse 
	// button
	GlobalResourceManager::use()->setMouseButtonInfo( button, state );

	if( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		//animTcl::OutputMessage("My mouse received a mouse button press event\n");

	}
	if( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{
		//animTcl::OutputMessage("My mouse received a mouse button release event\n") ;
		BaseSystem* hermiteBase = GlobalResourceManager::use()->getSystem("hermite");
		if (hermiteBase)
		{
			HermiteSystem* hermite = dynamic_cast<HermiteSystem*>(hermiteBase);
			Vector pos;
			pickFromXYPlane(pos, x, y);
			hermite->onLeftClick(pos);
		}

	}
}	// myMouse

// interaction (mouse motion)
void myMotion(int x, int y)
{

	GLMouseButtonInfo updatedMouseButtonInfo = 
		GlobalResourceManager::use()->getMouseButtonInfo();

	if( updatedMouseButtonInfo.button == GLUT_LEFT_BUTTON )
	{
		animTcl::OutputMessage(
			"My mouse motion callback received a mousemotion event\n") ;
	}

}	// myMotion

void pickFromXYPlane(Vector result, int x, int y)
{
	double modelView[16];
	double projection[16];
	int viewport[4];

	double x1, y1, z1, x2, y2, z2;

	glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	y = viewport[3] - y;
	gluUnProject(x, y, 0, modelView, projection, viewport, &x1, &y1,
		&z1);
	gluUnProject(x, y, 1, modelView, projection, viewport, &x2, &y2,
		&z2);

	double t = z1 / (z1 - z2);

	result[0] = (1 - t) * x1 + t * x2;
	result[1] = (1 - t) * y1 + t * y2;
	result[2] = 0;

	double z = (1 - t) * z1 + t * z2;
}

void MakeScene(void)
{

	/* 
	
	This is where you instantiate all objects, systems, and simulators and 
	register them with the global resource manager

	*/

	/* SAMPLE SCENE */


}	// MakeScene

// OpenGL initialization
void myOpenGLInit(void)
{
	animTcl::OutputMessage("Initialization routine was called.");

}	// myOpenGLInit

void myIdleCB(void)
{
	
	return;

}	// myIdleCB

void myKey(unsigned char key, int x, int y)
{
	animTcl::OutputMessage("My key callback received a key press event\n");

	return;

}	// myKey

static int testGlobalCommand(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	 animTcl::OutputMessage("This is a test command!");
	return TCL_OK;

}	// testGlobalCommand

static int partOneGlobalCommand(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	GlobalResourceManager::use()->clearAll();
	animTcl::OutputMessage("Objects cleaned!");
	glutPostRedisplay();

	bool success;

	// register a hermite system
	HermiteSystem* hermiteSystem1 = new HermiteSystem("hermite");
	success = GlobalResourceManager::use()->addSystem(hermiteSystem1, true);
	assert(success);
	animTcl::OutputMessage("Hermite system created!");

	// register a hermite simulator
	HermiteSimulator* hermiteSimulator = new HermiteSimulator("HermiteSimulator", hermiteSystem1);
	success = GlobalResourceManager::use()->addSimulator(hermiteSimulator, true);
	assert(success);
	animTcl::OutputMessage("Hermite simulator created!");

	// register a hermite curve
	Hermite* hermite1 = new Hermite("Hermite1");
	success = GlobalResourceManager::use()->addObject(hermite1, true);
	assert(success);
	animTcl::OutputMessage("Hermite curve created!");

	hermiteSystem1->setHermiteObject(hermite1);

	return TCL_OK;
}

static int partTwoGlobalCommand(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	GlobalResourceManager::use()->clearAll();
	animTcl::OutputMessage("Objects cleaned!");
	glutPostRedisplay();

	bool success;

	// register a tankpath system
	TankPathSystem* tankPathSystem = new TankPathSystem("tankpath");
	success = GlobalResourceManager::use()->addSystem(tankPathSystem, true);
	assert(success);
	animTcl::OutputMessage("Tankpath system created!");

	// register a tankpath simulator
	TankPathSimulator* tankPathSimulator = new TankPathSimulator("tankPathSimulator", tankPathSystem);
	success = GlobalResourceManager::use()->addSimulator(tankPathSimulator, true);
	assert(success);
	animTcl::OutputMessage("Tankpath simulator created!");

	// register a hermite curve for path
	Hermite* tankPathHermite = new Hermite("tankpathHermite");
	success = GlobalResourceManager::use()->addObject(tankPathHermite, true);
	assert(success);
	animTcl::OutputMessage("Tank path curve created!");

	// register a hermite curve for uniformed path
	Hermite* tankPathUniform = new Hermite("tankpathUniform");
	success = GlobalResourceManager::use()->addObject(tankPathUniform, true);
	assert(success);
	animTcl::OutputMessage("Tank path uniform curve created!");

	tankPathSystem->setPath(tankPathHermite);
	tankPathSystem->setUniformedPath(tankPathUniform);
	tankPathSystem->loadModel();

	// register a missile system
	MissileSystem* missileSystem = new MissileSystem("missile");
	success = GlobalResourceManager::use()->addSystem(missileSystem, true);
	assert(success);
	animTcl::OutputMessage("Missile system created!");

	// register a missile simulator
	MissileSimulator* missileSimulator = new MissileSimulator("missileSimulator", missileSystem);
	success = GlobalResourceManager::use()->addSimulator(missileSimulator, true);
	assert(success);
	animTcl::OutputMessage("Missile simulator created!");

	// register a hermite curve for missile path
	Hermite* missileHermite = new Hermite("missileHermite");
	success = GlobalResourceManager::use()->addObject(missileHermite, true);
	assert(success);
	animTcl::OutputMessage("Missile path created!");

	missileSystem->setPath(missileHermite);
	missileSystem->initializePath();
	missileSystem->loadModel();
	missileSimulator->setTargetTank(tankPathSimulator);
	glutPostRedisplay();

	return TCL_OK;
}

void mySetScriptCommands(Tcl_Interp *interp)
{

	// here you can register additional generic (they do not belong to any object) 
	// commands with the shell

	Tcl_CreateCommand(interp, "test", testGlobalCommand, (ClientData) NULL,
					  (Tcl_CmdDeleteProc *)	NULL);
	Tcl_CreateCommand(interp, "part1", partOneGlobalCommand, (ClientData)NULL,
			(Tcl_CmdDeleteProc *)NULL);
	Tcl_CreateCommand(interp, "part2", partTwoGlobalCommand, (ClientData)NULL,
		(Tcl_CmdDeleteProc *)NULL);

}	// mySetScriptCommands
