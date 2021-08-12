#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <opencv2/core/mat.hpp>
#include <vector>
#include "MapObject.h"
#include "AbstractTrajectory.h"

using namespace cv;
using namespace std;


class Segment : public MapObject
{
private:
protected:
	Scalar color = Scalar(1, 0, 1, 1);

	Segment() {};
	Segment(Map* map, int cvPointCount);
	virtual void detachAbstract() = 0;
public:
	virtual int antipattern() = 0;
	virtual const Point2f& getStart() = 0;
	virtual const Point2f& getEnd() = 0;
	virtual const Vec2f& getStartOri() = 0;
	virtual const float& getLength() = 0;
	inline void setColor(Scalar color) { this->color = color; }

	virtual void draw() const;

	virtual ~Segment() {};

	friend class Trajectory;
	friend class RamTreeNode;
};

class LinearSegment: public Segment
{
protected:
	LinearAbstractSegment* aSeg;

	void calculateCVPoints();
	LinearSegment() {};
	void detachAbstract();
public:
	LinearSegment(Map* map, LinearAbstractSegment* as);
	inline const Point2f& getStart() { return this->aSeg->start; }
	inline const Point2f& getEnd() { return this->aSeg->end; }
	inline const Vec2f& getStartOri() { return this->aSeg->startOri; }
	inline const float& getLength() { return this->aSeg->length; }
	inline const int& getDirection() { return this->aSeg->direction; }
	inline int antipattern() { return 0; };

	virtual ~LinearSegment();

	friend class Trajectory;
};

class CurveSegment : public Segment
{
private:
	CurveAbstractSegment* aSeg;
protected:
	void calculateCVPoints();
	CurveSegment() {};
	void detachAbstract();
public:
	CurveSegment(Map* map, CurveAbstractSegment* as, int cvPointCount = 10);
	inline const Point2f& getStart() { return this->aSeg->start; }
	inline const Point2f& getEnd() { return this->aSeg->end; }
	inline const Vec2f& getStartOri() { return this->aSeg->startOri; }
	inline const float& getLength() { return this->aSeg->length; }
	inline const float& getAngle() { return this->aSeg->angle; }
	inline const Vec2f& getEndOri() { return this->aSeg->endOri; }
	inline const float& getRadius() { return aSeg->radius; }
	inline const bool& isRight() { return aSeg->right; }
	inline int antipattern() { return 1; };

	virtual ~CurveSegment();

	friend class Trajectory;
};

class Trajectory : public MapObject
{
private:
	vector<Segment*> segments;
	AbstractTrajectory* aTraj;
protected:
	virtual void calculateCVPoints();

public:
	Trajectory(Map* map);
	Trajectory(Map* map, const Point2f& startPos, const Vec2f& startOri, float stepLength=1.0);
	virtual ~Trajectory();

	inline const Point2f getStartPos() { return this->aTraj->startPos; }
	inline const Point2f getEndPos() { return this->aTraj->endPos; }
	inline const Vec2f getStartOri() { return this->aTraj->startOri; }
	inline const Vec2f getEndOri() { return this->aTraj->endOri; }
	inline const float getLength() { return this->aTraj->length; }

	inline const Point2f getCurrPos() { return this->aTraj->currPos; }
	inline const Point2f getCurrOri() { return this->aTraj->currOri; }

	void setStepLength(float stepLength);
	void resetState();
	void addLinearSegment(float length, bool forward = true);
	void addCurveSegment(float angle, float radius, bool rigth);
	void removeLastSegment();
	void clear();
	bool step();
	void setColor(Scalar color);

	void draw() const;

	friend class Vehicle;
};

#endif // TRAJECTORY_H