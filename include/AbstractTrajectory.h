#ifndef ABSTRACT_TRAJECTORY_H
#define ABSTRACT_TRAJECTORY_H

#include <opencv2/core/mat.hpp>
#include <vector>

using namespace cv;
using namespace std;

class Segment;
class Map;

class AbstractSegment
{
protected:
	Point2f start;
	Point2f end;
	Vec2f startOri;
	float length;

	float currentSegmentPos = 0;
	float prevSegmentPos = 0;

	bool checkOverflow(float& stepLength);
public:

	inline const Point2f& getStart() { return this->start; }
	inline const Point2f& getEnd() { return this->end; }
	inline const Vec2f& getStartOri() { return this->startOri; }
	inline const float& getLength() { return this->length; }

	void resetState() { currentSegmentPos = 0; }
	virtual bool step(float& stepLength, Point2f& newPos, Vec2f& newOri) = 0;
	inline void restoreLastStep() { this->currentSegmentPos = this->prevSegmentPos; }
	virtual const Vec2f& getEndOri() = 0;
	virtual void truncateNow() = 0;
	virtual Segment* WTFNewSegment(Map* map) = 0;

	virtual ~AbstractSegment() {};

	friend class Segment;
};

class LinearAbstractSegment : public AbstractSegment
{
private:
	int direction;

public:
	LinearAbstractSegment(const Point2f& start, const Vec2f& ori, float length, bool forward = true);
	LinearAbstractSegment(const Point2f& start, const Point2f& end, bool forward = true);

	virtual bool step(float& stepLength, Point2f& newPos, Vec2f& newOri);
	const Vec2f& getEndOri() { return this->startOri; }
	int getDirection() { return this->direction; }
	void truncateNow();
	virtual Segment* WTFNewSegment(Map* map);

	friend class LinearSegment;
};

class CurveAbstractSegment : public AbstractSegment
{
private:
	float angle;
	float radius;
	bool right;
	Vec2f endOri;
	Point2f curveCenter;

public:
	CurveAbstractSegment(const Point2f& start, const Vec2f& startOri, float angle, float radius, bool right);

	inline const float& getAngle() { return this->angle; }
	inline const Vec2f& getEndOri() { return this->endOri; }
	inline const float& getRadius() { return this->radius; }
	inline const bool& isRight() { return this->right; }

	virtual bool step(float& stepLength, Point2f& newPos, Vec2f& newOri);
	void truncateNow();
	virtual Segment* WTFNewSegment(Map* map);

	friend class CurveSegment;
};

class AbstractTrajectory
{
private:
	Point2f startPos;
	Vec2f startOri;
	Point2f endPos;
	Vec2f endOri;

	Point2f currPos;
	Vec2f currOri;
	Point2f prevPos;
	Vec2f prevOri;

	vector<AbstractSegment*> segments;
	float length;
	float stepLength;
	int activeSegment;
	int prevActiveSegment;

	void truncateNow();
public:
	AbstractTrajectory(const Point2f& startPos, const Vec2f& startOri, float stepLength = 1.0);
	virtual ~AbstractTrajectory();

	inline const Point2f getStartPos() { return this->startPos; }
	inline const Point2f getEndPos() { return this->endPos; }
	inline const Vec2f getStartOri() { return this->startOri; }
	inline const Vec2f getEndOri() { return this->endOri; }
	inline const float getLength() { return this->length; }

	inline const Point2f getCurrPos() { return this->currPos; }
	inline const Point2f getCurrOri() { return this->currOri; }
	inline const Point2f getPrevPos() { return this->prevPos; }
	inline const Point2f getPrevOri() { return this->prevOri; }

	void setStepLength(float stepLength);
	void resetState();
	LinearAbstractSegment* addLinearSegment(float length, bool forward = true);
	CurveAbstractSegment* addCurveSegment(float angle, float radius, bool rigth);
	void removeLastSegment();
	void clear();
	bool step();
	void restoreLastStep();
	bool truncate(Map* map, float truncLength, bool& truncated, float stepsize, bool useChunk = false);

	friend class Trajectory;
	friend class RamTreeNode;
};

#endif // ABSTRACT_TRAJECTORY_H
