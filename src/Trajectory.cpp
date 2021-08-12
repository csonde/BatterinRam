#include <math.h>
#include "Trajectory.h"
#include "Map.h"
#include <opencv2/imgproc.hpp>

void Trajectory::calculateCVPoints()
{
	for (vector<Segment*>::iterator it = this->segments.begin(); it != this->segments.end(); it++)
	{
		(*it)->calculateCVPoints();
	}
}

Trajectory::Trajectory(Map* map) : Trajectory(map, Point2f(0,0), Vec2f(1,0))
{
}

Trajectory::Trajectory(Map* map, const Point2f& startPos, const Vec2f& startOri, float stepLength) :
	MapObject(map)
{
	this->aTraj = new AbstractTrajectory(startPos, startOri, stepLength);
	this->calculateCVPoints();
}

Trajectory::~Trajectory()
{
	delete this->aTraj;
	for (vector<Segment*>::iterator it = segments.begin(); it != segments.end(); it++)
		(*it)->detachAbstract();
	for (vector<Segment*>::iterator it = segments.begin(); it != segments.end(); it++)
		delete *it;
}

void Trajectory::setStepLength(float stepLength)
{
	this->aTraj->setStepLength(stepLength);
}

void Trajectory::resetState()
{
	this->aTraj->resetState();
}

void Trajectory::addLinearSegment(float length, bool forward)
{
	LinearAbstractSegment* aSeg = this->aTraj->addLinearSegment(length, forward);
	segments.push_back(new LinearSegment(this->map, aSeg));
	this->calculateCVPoints();
}

void Trajectory::addCurveSegment(float angle, float radius, bool right)
{
	CurveAbstractSegment* aSeg = this->aTraj->addCurveSegment(angle, radius, right);
	segments.push_back(new CurveSegment(this->map, aSeg));
	this->calculateCVPoints();
}

void Trajectory::removeLastSegment()
{
	if (!this->segments.size())
		return;

	this->resetState();

	this->aTraj->removeLastSegment();

	this->segments.pop_back();
	this->calculateCVPoints();
}

void Trajectory::clear()
{
	this->aTraj->clear();
	for (vector<Segment*>::iterator it = segments.begin(); it != segments.end(); it++)
		(*it)->detachAbstract();
	for (vector<Segment*>::iterator it = segments.begin(); it != segments.end(); it++)
		delete* it;
	this->calculateCVPoints();
}

bool Trajectory::step()
{
	return this->aTraj->step();
}

void Trajectory::setColor(Scalar color)
{
	for (vector<Segment*>::iterator it = this->segments.begin(); it != segments.end(); it++)
	{
		(*it)->setColor(color);
	}
}

void Trajectory::draw() const
{
	for (vector<Segment*>::const_iterator it = this->segments.begin(); it != this->segments.end(); it++)
	{
		(*it)->draw();
	}
}

void LinearSegment::calculateCVPoints()
{
	this->cvPoints[0] = Point2i((int)(this->aSeg->start.x * this->map->getScale() + this->map->getOffsetX()), (int)(this->aSeg->start.y * this->map->getScale() + this->map->getOffsetY()));
	this->cvPoints[1] = Point2i((int)(this->aSeg->end.x * this->map->getScale() + this->map->getOffsetX()), (int)(this->aSeg->end.y * this->map->getScale() + this->map->getOffsetY()));
}

LinearSegment::LinearSegment(Map* map, LinearAbstractSegment* as) : Segment(map, 2), aSeg(as)
{
	this->calculateCVPoints();
}

void LinearSegment::detachAbstract()
{
	this->aSeg = 0;
}

LinearSegment::~LinearSegment()
{
	if (this->aSeg)
		delete this->aSeg;
}

void CurveSegment::calculateCVPoints()
{
	for (int i = 0; i < this->cvPointCount; ++i)
	{
		Point2f realPoint;
		float angle = (float)i / ((float)this->cvPointCount -1) * this->aSeg->angle;

		realPoint.x = (this->aSeg->start.x - this->aSeg->curveCenter.x) * cos(angle) -
			(this->aSeg->start.y - this->aSeg->curveCenter.y) * sin(angle) + this->aSeg->curveCenter.x;
		realPoint.y = (this->aSeg->start.x - this->aSeg->curveCenter.x) * sin(angle) +
			(this->aSeg->start.y - this->aSeg->curveCenter.y) * cos(angle) + this->aSeg->curveCenter.y;

		this->cvPoints[i] = Point2i((int)(realPoint.x * this->map->getScale() + this->map->getOffsetX()), (int)(realPoint.y * this->map->getScale() + this->map->getOffsetY()));
	}
}

CurveSegment::CurveSegment(Map* map, CurveAbstractSegment* as, int cvPointCount) : Segment(map, cvPointCount), aSeg(as)
{
	this->calculateCVPoints();
}

void CurveSegment::detachAbstract()
{
	this->aSeg = 0;
}

CurveSegment::~CurveSegment()
{
	if (this->aSeg)
		delete this->aSeg;
}

Segment::Segment(Map* map, int cvPointCount) : MapObject(map,cvPointCount)
{
}

void Segment::draw() const
{
	const Point2i* pts = this->cvPoints; 
	int npt = this->cvPointCount;
	polylines(this->map->map, &pts, &npt, 1, false, this->color, 2);
}
