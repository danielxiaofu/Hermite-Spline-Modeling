////////////////////////////////////////////////////
// Hermite spline class
// Created by Yixiao Yang
////////////////////////////////////////////////////

#pragma once

#include <vector>
#include <map>
#include <util/util.h>
#include "shared/opengl.h"
#include "BaseObject.h"

struct ControlPoint
{
	Vector point;
	Vector tangent;

	ControlPoint()
	{
		zeroVector(point);
		zeroVector(tangent);
	}

	ControlPoint(double px, double py, double pz, double tx, double ty, double tz)
	{
		setVector(point, px, py, pz);
		setVector(tangent, tx, ty, tz);
	}

	ControlPoint(Vector position, Vector pointTangent)
	{
		VecCopy(point, position);
		VecCopy(tangent, pointTangent);
	}

	void getPoint(Vector v)
	{
		VecCopy(v, point);
	}

	void getTangent(Vector v)
	{
		VecCopy(v, tangent);
	}

};

struct Segment
{
	Vector a = { 0, 0, 0 };
	Vector b = { 0, 0, 0 };
	Vector c = { 0, 0, 0 };
	Vector d = { 0, 0, 0 };
	Vector y0 = { 0, 0, 0 }; // start point
	Vector y1 = { 0, 0, 0 }; // end tangent
	Vector s0 = { 0, 0, 0 }; // start tangent
	Vector s1 = { 0, 0, 0 }; // end tangent

	Segment();

	Segment(ControlPoint startPoint, ControlPoint endPoint);

	void setStartPoint(ControlPoint startPoint);

	void setEndPoint(ControlPoint endPoint);

	void computeCurve();

	void getPosition(Vector result, double t);

	void getTangent(Vector result, double t);
};

class Hermite : public BaseObject
{

protected:
	const int numOfSamples = 30; // number of sample lines to describe a curve segment
	const float lengthDeltaT = 0.05;

	std::vector<ControlPoint> controlPoints;
	std::vector<Segment> segments;
	std::map<int, double> lengthMap;

public:
	Hermite( const std::string& name );

	void addControlPoint(ControlPoint controlPoint);

	void addControlPoint(double px, double py, double pz, double sx, double sy, double sz);

	void addControlPoint(double px, double py, double pz);

	void display(GLenum mode = GL_RENDER);

	bool setPoint(int index, double x, double y, double z);

	bool setTangent(int index, double x, double y, double z);

	void getPosition(Vector result, double t);

	void generateLengthTable();

	double getArcLength(double t);

	void crInitialize();

protected:

	void addSegment(Segment segment);

	// given index of a controlpoint, recompute the segments that connect that point
	void recomputeSegment(int pointIndex);
};