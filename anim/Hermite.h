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

	const float lengthDeltaT = 0.05;

	std::map<int, double> lengthMap;

	void setStartPoint(ControlPoint startPoint);

	void setEndPoint(ControlPoint endPoint);

	void getPosition(Vector result, double t);

	void getTangent(Vector result, double t);

	void generateLengthTable();

	double getArcLength(double t);

	void computeCurve();

	double getPointFromLength(double targetLength);

	// perform binary search on length table, return t value that related to targetLength
	double binarySearchLength(double startT, double endT, double targetLength);
};

class Hermite : public BaseObject
{

protected:
	const int numOfSamples = 30; // number of sample lines to describe a curve segment, used in rendering
	const float lengthDeltaT = 0.05;

	std::vector<ControlPoint> controlPoints;
	std::vector<Segment> segments;
	std::map<int, double> lengthMap;

public:
	Hermite( const std::string& name );

	bool visible = true;

	Vector color;

	void display(GLenum mode = GL_RENDER);

	void reset(double time);

	// modification function

	void addControlPoint(ControlPoint controlPoint);

	void addControlPoint(double px, double py, double pz, double sx, double sy, double sz);

	void addControlPoint(double px, double py, double pz);

	bool setPoint(int index, double x, double y, double z);

	bool setTangent(int index, double x, double y, double z);

	// end of modification function

	// query function

	void getPosition(Vector result, double t);

	void getTangent(Vector result, double t);

	int getNumPoints() const;

	void getControlPoint(Vector p, int index);

	void getControlPointTangent(Vector p, int index);

	double getArcLength(double t);

	void getPointFromLength(Vector position, Vector tangent, double arcLength);

	// end of query function

	// other function

	void generateLengthTable();

	void applyCR();

	void turnOffCR();

	// construct a uniform version of this curve, uniform means linear relationship between arclength and t
	//void generateUniformCurve(Hermite* result);

	// end of other function

protected:

	void addSegment(Segment segment);

	// given index of segment, return sum of length of all segments before this segment
	double getGlobalLength(int segmentIndex);

	// given index of a controlpoint, recompute the segments that connect that point
	void recomputeSegment(int pointIndex);

	// perform binary search on length table, return t value that related to targetLength
	double binarySearchLength(double startT, double endT, double targetLength);
};