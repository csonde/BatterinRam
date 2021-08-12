#include "AbstractTrajectory.h"
#include "Trajectory.h"

#include <math.h>
#include <opencv2/imgproc.hpp>
#include "Map.h"

AbstractTrajectory::AbstractTrajectory(const Point2f& startPos, const Vec2f& startOri, float stepLength) :
	startPos(startPos),
	startOri(startOri),
	endPos(startPos),
	endOri(startOri),
	currPos(startPos),
	currOri(startOri),
	segments(),
	length(0),
	stepLength(stepLength),
	activeSegment(-1),
	prevActiveSegment(-1)
{
}

AbstractTrajectory::~AbstractTrajectory()
{
	for (vector<AbstractSegment*>::iterator it = this->segments.begin(); it != this->segments.end(); it++)
		delete* it;
}

void AbstractTrajectory::setStepLength(float stepLength)
{
	this->resetState();
	this->stepLength = stepLength;
}

void AbstractTrajectory::resetState()
{
	this->currPos = this->startPos;
	this->currOri = this->startOri;
	if (this->segments.size())
		this->prevActiveSegment = this->activeSegment = 0;
	for (vector<AbstractSegment*>::iterator it = this->segments.begin(); it != this->segments.end(); it++)
		(*it)->resetState();
}

LinearAbstractSegment* AbstractTrajectory::addLinearSegment(float length, bool forward)
{
	LinearAbstractSegment* s = new LinearAbstractSegment(this->endPos, this->endOri, length, forward);
	this->segments.push_back(s);
	this->resetState();
	this->endPos = s->getEnd();
	this->length += length;
	return s;
}

CurveAbstractSegment* AbstractTrajectory::addCurveSegment(float angle, float radius, bool right)
{
	CurveAbstractSegment* s = new CurveAbstractSegment(this->endPos, this->endOri, angle, radius, right);
	segments.push_back(s);
	this->resetState();
	this->endPos = s->getEnd();
	this->endOri = s->getEndOri();
	this->length += s->getLength();
	return s;
}

void AbstractTrajectory::removeLastSegment()
{
	if (!segments.size())
		return;

	this->resetState();

	if (segments.size() == 1)
	{
		this->endPos = this->startPos;
		this->endOri = this->startOri;
		this->length = 0;
		this->prevActiveSegment = this->activeSegment = -1;
	}
	else
	{
		this->endPos = segments[segments.size()]->getStart();
		this->endOri = segments[segments.size()]->getStartOri();
		this->length -= segments[segments.size()]->getLength();
	}

	segments.pop_back();
}

void AbstractTrajectory::clear()
{
	this->endPos = startPos;
	this->endOri = startOri;
	this->currPos = this->startPos;
	this->currOri = startOri;
	this->length = 0;
	this->prevActiveSegment = this->activeSegment = -1;
	for (vector<AbstractSegment*>::iterator it = segments.begin(); it != segments.end(); it++)
		delete* it;
	this->segments.clear();
}

bool AbstractTrajectory::step()
{
	if (this->activeSegment == -1)
		return true;

	this->prevActiveSegment = this->activeSegment;
	this->prevPos = this->currPos;
	this->prevOri = this->currOri;

	float currStepLength = this->stepLength;

	while (this->activeSegment < this->segments.size())
	{
		if (this->segments[this->activeSegment]->step(currStepLength, this->currPos, this->currOri))
			++(this->activeSegment);
		else
			break;
	}

	if (this->activeSegment == this->segments.size())
	{
		this->currPos = this->endPos;
		this->currOri = this->endOri;
		return true;
	}

	return false;
}

void AbstractTrajectory::restoreLastStep()
{
	Point2f pos = this->prevPos;
	Vec2f ori = this->prevOri;
	int segInd = this->prevActiveSegment;
	this->resetState();
	this->currPos = pos;
	this->currOri = ori;
	this->activeSegment = segInd;
	if (this->activeSegment >= 0)
		this->segments[activeSegment]->restoreLastStep();
}

void AbstractTrajectory::truncateNow()
{
	if (this->activeSegment < 0)
		return;

	if(this->segments.size() > this->activeSegment + 1)
		this->segments.erase(this->segments.begin() + this->activeSegment + 1, this->segments.end());
	this->segments[this->activeSegment]->truncateNow();
	this->endPos = this->segments[this->activeSegment]->getEnd();
	this->endOri = this->segments[this->activeSegment]->getEndOri();
	this->length = 0;
	for (vector<AbstractSegment*>::iterator it = this->segments.begin(); it != this->segments.end(); it++)
		this->length += (*it)->getLength();
	this->resetState();
}

bool AbstractTrajectory::truncate(Map* map, float truncLength, bool& truncated, float stepsize, bool useChunk)
{
	this->resetState();
	float currLen = 0;
	bool midSection = true;
	truncated = false;
	while ((truncLength < 0 || currLen < truncLength) && (midSection = !this->step()))
	{
		if (map->checkCollision(this->currPos, this->currOri, stepsize))
		{
			truncated = true;
			if (useChunk && currLen > 0)
			{
				this->restoreLastStep();
				break;
			}
			return false;
		}
		currLen += stepsize;
	}
	if (midSection)
	{
		this->truncateNow();
		truncated = true;
	}
	return true;
}

LinearAbstractSegment::LinearAbstractSegment(const Point2f& start, const Vec2f& ori, float length, bool forward)
{
	this->start = start;
	this->startOri = normalize(ori);
	this->length = length;
	this->direction = forward ? 1 : -1;

	this->end = (Vec2f)this->start + this->direction * this->length * this->startOri;
}

LinearAbstractSegment::LinearAbstractSegment(const Point2f& start, const Point2f& end, bool forward)
{
	this->start = start;
	this->end = end;
	this->startOri = normalize((Vec2f)(this->end - this->start));
	this->length = norm(this->end - this->start);
	this->direction = forward ? 1 : -1;
}

bool LinearAbstractSegment::step(float& stepLength, Point2f& newPos, Vec2f& newOri)
{
	if (checkOverflow(stepLength))
		return true;

	this->prevSegmentPos = currentSegmentPos;

	this->currentSegmentPos += stepLength;
	newPos = (Vec2f)this->start + this->direction * this->currentSegmentPos * this->startOri;
	newOri = this->startOri;
	return false;
}

void LinearAbstractSegment::truncateNow()
{
	this->end = (Vec2f)this->start + this->startOri * this->direction * this->currentSegmentPos;
	this->length = this->currentSegmentPos;
}

Segment* LinearAbstractSegment::WTFNewSegment(Map* map)
{
	return new LinearSegment(map, new LinearAbstractSegment(*this));
}

CurveAbstractSegment::CurveAbstractSegment(const Point2f& start, const Vec2f& startOri, float angle, float radius, bool right) : radius(radius), right(right)
{
	this->start = start;
	this->startOri = normalize(startOri);
	this->length = abs(angle) * radius;
	Vec2f centerDir;
	if (right)
	{
		centerDir[0] = this->startOri[1];
		centerDir[1] = -this->startOri[0];
	}
	else
	{
		centerDir[0] = -this->startOri[1];
		centerDir[1] = this->startOri[0];
	}
	this->angle = angle;
	this->curveCenter = (Vec2f)start + centerDir * radius;

	this->end.x = (this->start.x - this->curveCenter.x) * cos(angle) -
		(this->start.y - this->curveCenter.y) * sin(angle) + this->curveCenter.x;
	this->end.y = (this->start.x - this->curveCenter.x) * sin(angle) +
		(this->start.y - this->curveCenter.y) * cos(angle) + this->curveCenter.y;

	this->endOri[0] = this->startOri[0] * cos(angle) - this->startOri[1] * sin(angle);
	this->endOri[1] = this->startOri[0] * sin(angle) + this->startOri[1] * cos(angle);
	this->endOri = normalize(endOri);
}

bool CurveAbstractSegment::step(float& stepLength, Point2f& newPos, Vec2f& newOri)
{
	if (this->checkOverflow(stepLength))
		return true;

	this->prevSegmentPos = currentSegmentPos;

	this->currentSegmentPos += stepLength;
	float currAngle = this->angle * this->currentSegmentPos / this->length;

	newPos.x = (this->start.x - this->curveCenter.x) * cos(currAngle) -
		(this->start.y - this->curveCenter.y) * sin(currAngle) + this->curveCenter.x;
	newPos.y = (this->start.x - this->curveCenter.x) * sin(currAngle) +
		(this->start.y - this->curveCenter.y) * cos(currAngle) + this->curveCenter.y;

	newOri[0] = this->startOri[0] * cos(currAngle) - this->startOri[1] * sin(currAngle);
	newOri[1] = this->startOri[0] * sin(currAngle) + this->startOri[1] * cos(currAngle);

	return false;
}

void CurveAbstractSegment::truncateNow()
{
	this->angle *= this->currentSegmentPos / this->length;

	this->end.x = (this->start.x - this->curveCenter.x) * cos(this->angle) -
		(this->start.y - this->curveCenter.y) * sin(this->angle) + this->curveCenter.x;
	this->end.y = (this->start.x - this->curveCenter.x) * sin(this->angle) +
		(this->start.y - this->curveCenter.y) * cos(this->angle) + this->curveCenter.y;

	this->endOri[0] = this->startOri[0] * cos(this->angle) - this->startOri[1] * sin(this->angle);
	this->endOri[1] = this->startOri[0] * sin(this->angle) + this->startOri[1] * cos(this->angle);

	this->length = this->currentSegmentPos;
}

Segment* CurveAbstractSegment::WTFNewSegment(Map* map)
{
	return new CurveSegment(map, new CurveAbstractSegment(*this));
}

bool AbstractSegment::checkOverflow(float& stepLength)
{
	if (stepLength > this->length - this->currentSegmentPos)
	{
		stepLength = stepLength - this->length + this->currentSegmentPos;
		return true;
	}
	return false;
}