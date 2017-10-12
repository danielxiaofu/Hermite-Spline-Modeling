#include "Hermite.h"

Segment::Segment()
{
}

Segment::Segment(ControlPoint startPoint, ControlPoint endPoint)
{
	setStartPoint(startPoint);
	setEndPoint(endPoint);
}

void Segment::setStartPoint(ControlPoint startPoint)
{
	VecCopy(y0, startPoint.point);
	VecCopy(s0, startPoint.tangent);
	computeCurve();
	generateLengthTable();
}

void Segment::setEndPoint(ControlPoint endPoint)
{
	VecCopy(y1, endPoint.point);
	VecCopy(s1, endPoint.tangent);
	computeCurve();
	generateLengthTable();
}

void Segment::computeCurve()
{
	VecCopy(d, y0); // d = y0
	VecCopy(c, s0); // c = s0
	// a = -2(y1 - y0) + s0 + s1
	Vector temp;
	VecSubtract(temp, y1, y0);
	VecScale(temp, -2);
	VecAdd(temp, temp, s0);
	VecAdd(a, temp, s1);
	// b = 3(y1 - y0) - 2 * s0 - s1
	VecSubtract(temp, y1, y0);
	VecScale(temp, 3);
	Vector temp2;
	VecCopy(temp2, s0);
	VecScale(temp2, 2);
	VecSubtract(temp, temp, temp2);
	VecSubtract(b, temp, s1);
}

double Segment::getPointFromLength(double arcLength)
{
	if (arcLength >= getArcLength(1))
		return 1.0;
	else if (fabs(arcLength) <= DBL_EPSILON)
	{
		return 0.0;
	}

	double t = binarySearchLength(0.0, 1.0, arcLength);

	return t;
}

double Segment::binarySearchLength(double startT, double endT, double targetLength)
{
	// get the arc length and table index at middle point
	double middleT = (startT + endT) / 2;

	//animTcl::OutputMessage("entry = %d", entry);

	double middleLength = getArcLength(middleT);
	double finalT = 0.0;

	if (fabs(middleLength - targetLength) < 0.01)
		return middleT;

	// otherwise do recursive root finding
	if (targetLength <= middleLength)
		finalT = binarySearchLength(startT, middleT, targetLength);
	else
		finalT = binarySearchLength(middleT, endT, targetLength);

	return finalT;
}

void Segment::getPosition(Vector result, double t)
{
	if (t <= 0)
		VecCopy(result, y0);
	else if ( t > 0 && t < 1)
	{
		Vector aTemp, bTemp, cTemp;
		VecCopy(aTemp, a);
		VecCopy(bTemp, b);
		VecCopy(cTemp, c);
		VecScale(aTemp, t * t * t);
		VecScale(bTemp, t * t);
		VecScale(cTemp, t);
		VecAdd(result, aTemp, bTemp);
		VecAdd(result, result, cTemp);
		VecAdd(result, result, d);
	} else
		VecCopy(result, y1);
}

void Segment::getTangent(Vector result, double t)
{
	if (t <= 0)
		VecCopy(result, s0);
	else if (t > 0 && t < 1)
	{
		Vector aTemp, bTemp;
		VecCopy(aTemp, a);
		VecCopy(bTemp, b);
		VecScale(aTemp, 3 * t * t);
		VecScale(bTemp, 2 * t);
		VecAdd(result, aTemp, bTemp);
		VecAdd(result, result, c);
	}
	else
		VecCopy(result, s1);
}

void Segment::generateLengthTable()
{
	lengthMap.clear();

	Vector temp1, temp2, temp3;

	double tEntry = 0;
	double cumulateL = 0;

	lengthMap[0] = 0;
	tEntry += lengthDeltaT;
	getPosition(temp1, 0);

	while (tEntry < 1.0)
	{
		getPosition(temp2, tEntry);
		VecSubtract(temp3, temp2, temp1);
		double l = VecLength(temp3);
		cumulateL += l;
		lengthMap[(int)(tEntry / lengthDeltaT)] = cumulateL;
		tEntry += lengthDeltaT;
		VecCopy(temp1, temp2);
	}

	getPosition(temp3, 1);
	VecSubtract(temp3, temp3, temp2);
	double l = VecLength(temp3);
	cumulateL += l;
	lengthMap[(int)(1 / lengthDeltaT)] = cumulateL;

}

double Segment::getArcLength(double t)
{
	int entry = (int)(t / lengthDeltaT);
	std::map<int, double>::iterator it = lengthMap.find(entry);

	if (it != lengthMap.end())
	{
		double result = it->second + (t - entry * lengthDeltaT) / lengthDeltaT * (std::next(it)->second - it->second);
		return result;
	}

	return -1.0;

	return 0.0;
}



Hermite::Hermite(const std::string& name) :
	BaseObject(name)
{
	setVector(color, 1, 1, 1);
}

void Hermite::addControlPoint(ControlPoint controlPoint)
{

	if (controlPoints.size() >= 40)
	{
		animTcl::OutputMessage("Add control point failed, maximum control points reached");
		return;
	}

	if (controlPoints.size() > 0)
	{
		ControlPoint last = controlPoints.back();
		addSegment(Segment(last, controlPoint));
	}
	controlPoints.push_back(controlPoint);
	
}

void Hermite::addControlPoint(double px, double py, double pz, double sx, double sy, double sz)
{
	addControlPoint(ControlPoint(px, py, pz, sx, sy, sz));
}

// only positions are provided, compute tangent using Catmull-Rom initialization
void Hermite::addControlPoint(double px, double py, double pz)
{
	if (controlPoints.size() >= 40)
	{
		animTcl::OutputMessage("Add control point failed, maximum control points reached");
		return;
	}

	if (controlPoints.size() == 0) // no point
	{
		addControlPoint(ControlPoint(px, py, pz, 0, 0, 0));
	}
	else if (controlPoints.size() == 1) // one point
	{
		Vector point;
		setVector(point, px, py, pz);
		Vector tangent;
		VecSubtract(tangent, point, controlPoints[0].point);
		VecScale(tangent, 0.5);
		addControlPoint(ControlPoint(point, tangent));
	}
	else // more than one point
	{
		// sn-1 = 2(yn-1 - yn-2) - (yn-1 - yn-3)/2
		int n = controlPoints.size();
		Vector lastTangent, secondLast, point, tangent;
		setVector(point, px, py, pz);
		/*setVector(point, px, py, pz);
		VecSubtract(temp1, point, controlPoints[n - 1].point);
		VecScale(temp1, 2);
		VecSubtract(temp2, point, controlPoints[n - 2].point);
		VecScale(temp2, 0.5);
		VecSubtract(tangent, temp1, temp2);*/
		//VecSubtract(tangent, point, controlPoints[n - 1].point);
		zeroVector(tangent);
		controlPoints[n - 1].getTangent(lastTangent);
		controlPoints[n - 2].getPoint(secondLast);
		VecSubtract(lastTangent, point, secondLast);
		VecScale(lastTangent, 0.5);
		setTangent(n - 1, lastTangent[0], lastTangent[1], lastTangent[2]);
		addControlPoint(ControlPoint(point, tangent));
		

	}
}

void Hermite::display(GLenum mode)
{
	if (!visible)
		return;

	for (Segment segment : segments)
	{
		Vector p1, p2;
		segment.getPosition(p2, 0);

		for (int i = 1; i < numOfSamples; i++)
		{
			VecCopy(p1, p2);
			segment.getPosition(p2, i / (double)(numOfSamples - 1));
			glLineWidth(5);
			glColor3d(color[0], color[1], color[2]);
			glBegin(GL_LINES);
			glVertex3d(p1[0], p1[1], p1[2]);
			glVertex3d(p2[0], p2[1], p2[2]);
			glEnd();
		}
	}

	// draw controlpoints
	/*for (ControlPoint controlPoint : controlPoints)
	{
		Vector p;
		VecCopy(p, controlPoint.point);
		glPointSize(8);
		glBegin(GL_POINTS);
		glVertex3d(p[0], p[1], p[2]);
		glEnd();
	}*/
}

void Hermite::reset(double time)
{
	controlPoints.clear();
	segments.clear();
	lengthMap.clear();
}

bool Hermite::setPoint(int index, double x, double y, double z)
{
	if (index >= controlPoints.size())
		return false;
	Vector newPoint;
	newPoint[0] = x;
	newPoint[1] = y;
	newPoint[2] = z;

	VecCopy(controlPoints[index].point, newPoint);
	recomputeSegment(index);
	//generateLengthTable();
	return true;
}

bool Hermite::setTangent(int index, double x, double y, double z)
{
	if (index >= controlPoints.size())
		return false;
	Vector newTangent;
	newTangent[0] = x;
	newTangent[1] = y;
newTangent[2] = z;

VecCopy(controlPoints[index].tangent, newTangent);
recomputeSegment(index);
//generateLengthTable();
return true;
}

void Hermite::getPosition(Vector result, double t)
{
	if (segments.size() == 0) {
		zeroVector(result);
		return;
	}

	if (t >= 0 && t < 1)
	{
		double segmentLength = 1.0f / segments.size();
		int segmentIndex = (int)(t * (segments.size()));
		double localT = (t - (int)(t / segmentLength) * segmentLength) / segmentLength;
		segments[segmentIndex].getPosition(result, localT);
	}
	else
	{
		VecCopy(result, segments[segments.size() - 1].y1);
	}
	


	//animTcl::OutputMessage("segment index = %d, local t = %f", segmentIndex, localT);

}

void Hermite::getTangent(Vector result, double t)
{
	if (segments.size() == 0) {
		zeroVector(result);
		return;
	}

	if (t <= 0)
		VecCopy(result, segments[0].s0);
	else if (t > 0 && t < 1)
	{
		double segmentLength = 1.0f / segments.size();
		int segmentIndex = (int)(t * segments.size());
		double localT = abs(remainder(t, segmentLength));
		segments[segmentIndex].getTangent(result, localT);
		//animTcl::OutputMessage("local T = %f", localT);
	}
	else
		VecCopy(result, segments[segments.size() - 1].s1);
}

void Hermite::generateLengthTable()
{
	double tEntry = 0;

	// first entry (t is 0);
	lengthMap[0] = 0;
	tEntry = 0 + lengthDeltaT;

	double segmentLength = 1.0f / segments.size();
	while (tEntry < 1.0f)
	{
		int segmentIndex = (int)(tEntry * (segments.size()));
		double localT = (tEntry - (int)(tEntry / segmentLength) * segmentLength) / segmentLength;
		double length = getGlobalLength(segmentIndex) + segments[segmentIndex].getArcLength(localT);
		lengthMap[(int)(tEntry / lengthDeltaT)] = length;
		animTcl::OutputMessage("entry = %d t = %f length = %f", (int)(tEntry / lengthDeltaT), tEntry, length);
		tEntry += lengthDeltaT;
	}

	// last entry (t is 1)
	double last = getGlobalLength(segments.size() - 1) + segments[segments.size() - 1].getArcLength(1.0);
	lengthMap[(int)(1 / lengthDeltaT)] = last;
	animTcl::OutputMessage("entry = 1 t = 1 length = %f", last);
}

double Hermite::getArcLength(double t)
{
	//int entry = (int)(t / lengthDeltaT + 0.5);
	//animTcl::OutputMessage("entry = %d", entry);
	int entry = (int)(t / lengthDeltaT);
	std::map<int, double>::iterator it = lengthMap.find(entry);

	if (it != lengthMap.end() && std::next(it) != lengthMap.end())
	{
		double result = it->second + (t - entry * lengthDeltaT) / lengthDeltaT * (std::next(it)->second - it->second);
		return result;
	}

	return -1.0;
}

void Hermite::getPointFromLength(Vector position, Vector tangent, double arcLength)
{
	if (controlPoints.size() == 0)
	{
		zeroVector(position);
		zeroVector(tangent);
		return;
	}

	if (arcLength >= getArcLength(1))
	{
		segments[segments.size() - 1].getPosition(position, 1.0);
		segments[segments.size() - 1].getTangent(tangent, 1.0);
		return;
	}

	// go through the global length of every segment in order to find which segment the arcLength lands on
	double globalLength = 0;
	int i;
	int index = segments.size() - 1;
	bool stop = false;
	for (i = 0; !stop && i <= segments.size(); i++)
	{
		double nextLength = getGlobalLength(i);
		if (nextLength > arcLength)
		{
			stop = true;
			index = i - 1;
		}
		else
			globalLength = nextLength;
	}
	double localLength = arcLength - globalLength;
	double localT = segments[index].getPointFromLength(localLength);
	segments[index].getPosition(position, localT);
	segments[index].getTangent(tangent, localT);
}

int Hermite::getNumPoints() const
{
	return controlPoints.size();
}

void Hermite::getControlPoint(Vector p, int index)
{
	if (index >= 0 || index < controlPoints.size())
		controlPoints[index].getPoint(p);
	else
		zeroVector(p);
}

void Hermite::getControlPointTangent(Vector p, int index)
{
	if (index >= 0 || index < controlPoints.size())
		controlPoints[index].getTangent(p);
	else
		zeroVector(p);
}

void Hermite::applyCR()
{
	if (controlPoints.size() < 3)
	{
		animTcl::OutputMessage("Unable to apply CR-Initialize, less than 3 control points");
		return;
	}

	// s0 = 2(y1 - y0) - (y2 - y0)/2
	Vector temp1;
	Vector temp2;
	Vector newTangent;
	VecSubtract(temp1, controlPoints[1].point, controlPoints[2].point);
	VecScale(temp1, 2);
	VecSubtract(temp2, controlPoints[2].point, controlPoints[0].point);
	VecScale(temp2, 0.5);
	VecSubtract(newTangent, temp1, temp2);
	segments[0].setStartPoint(ControlPoint(controlPoints[0].point, newTangent));

	// si = (yi+1 - yi-1)/2 for i = 1,...,n-2
	for (int i = 1; i < controlPoints.size() - 1; i++)
	{
		VecSubtract(temp1, controlPoints[i + 1].point, controlPoints[i - 1].point);
		VecScale(temp1, 0.5);
		VecCopy(newTangent, temp1);
		segments[i - 1].setEndPoint(ControlPoint(controlPoints[i].point, newTangent));
		segments[i].setStartPoint(ControlPoint(controlPoints[i].point, newTangent));
	}

	// sn-1 = 2(yn-1 - yn-2) - (yn-1 - yn-3)/2
	int n = controlPoints.size();
	VecSubtract(temp1, controlPoints[n - 1].point, controlPoints[n - 2].point);
	VecScale(temp1, 2);
	VecSubtract(temp2, controlPoints[n - 1].point, controlPoints[n - 3].point);
	VecScale(temp2, 0.5);
	VecSubtract(newTangent, temp1, temp2);
	segments[n - 2].setEndPoint(ControlPoint(controlPoints[n - 1].point, newTangent));

}

void Hermite::turnOffCR()
{
	for (int i = 0; i < controlPoints.size(); i++)
	{
		recomputeSegment(i);
	}
}

//void Hermite::generateUniformCurve(Hermite * result)
//{
//	// clear all data of result curve
//	result->controlPoints.clear();
//	result->segments.clear();
//	result->lengthMap.clear();
//
//	// compute delta length for each uniform segment
//	double length = getArcLength(1);
//	double deltaLength = length / segments.size();
//
//	// create control points for new curve
//	double cumulateLength = 0;
//	Vector point, tangent;
//	for (int i = 0; i < segments.size() - 1; i++)
//	{
//		double t = getPointFromLength(point, tangent, cumulateLength);
//		result->addControlPoint(ControlPoint(point, tangent));
//		cumulateLength += deltaLength;
//		animTcl::OutputMessage("length = %f", cumulateLength);
//	}
//	// add the last point
//	getPointFromLength(point, tangent, cumulateLength);
//	result->addControlPoint(ControlPoint(point, tangent));
//	result->generateLengthTable();
//}

void Hermite::addSegment(Segment segment)
{
	segments.push_back(segment);
	//generateLengthTable();
}

double Hermite::getGlobalLength(int segmentIndex)
{
	if (segmentIndex < 0)
		return 0.0;
	else if (segmentIndex >= segments.size())
		segmentIndex = segments.size();

	double globalLength = 0;
	for (int i = segmentIndex - 1; i >= 0; i--)
	{
		globalLength += segments[i].getArcLength(1.0);
	}

	return globalLength;
}

void Hermite::recomputeSegment(int pointIndex)
{
	// not the first point, compute the segment before point 
	if (pointIndex > 0)
		segments[pointIndex - 1].setEndPoint(controlPoints[pointIndex]);
	// not the last point, compute the segment after point
	if (pointIndex < controlPoints.size() - 1)
		segments[pointIndex].setStartPoint(controlPoints[pointIndex]);
}

double Hermite::binarySearchLength(double startT, double endT, double targetLength)
{
	// get the arc length and table index at middle point
	double middleT = (startT + endT) / 2;

	//animTcl::OutputMessage("entry = %d", entry);

	double middleLength = getArcLength(middleT);
	double finalT = 0.0;

	if (fabs(middleLength - targetLength) < 0.01)
		return middleT;

	// otherwise do recursive root finding
	if (targetLength <= middleLength)
		finalT = binarySearchLength(startT, middleT, targetLength);
	else
		finalT = binarySearchLength(middleT, endT, targetLength);

	return finalT;
}
