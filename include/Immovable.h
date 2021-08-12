#ifndef IMMOVABLE_H
#define IMMOVABLE_H

#include <opencv2/core/mat.hpp>
#include <vector>
#include "MapObject.h"

using namespace cv;
using namespace std;

class AbstractTrajectory;

struct CarConfiguration
{
	CarConfiguration* parent;
	Point2f pos;
	Vec2f ori;
	AbstractTrajectory* t;
};

class Immovable : public MapObject
{
protected:
	vector<Point2f> points;
	Point2f centroid;
	float range;

	virtual void calculateCVPoints();
public:
	enum ObjectType
	{
		PARKING_SPOT,
		PILLAR,
		WALL,
	};

	Immovable(Map* map, const vector<float>& coords, int rowNum);
	inline Point2f getCentroid() { return this->centroid; }
	inline float getRange() { return this->range; }
	virtual ~Immovable();

	friend class Map;

	virtual bool checkCollision(Point2f collZoneCorners[4]);
};

class Pillar : public Immovable
{
public:
	Pillar(Map* map, const vector<float>& coords);
	virtual ~Pillar() {};

	virtual void draw() const;
};

class Wall : public Immovable
{
public:
	Wall(Map* map, const vector<float>& coords);
	virtual ~Wall() {};
	virtual void draw() const;
	bool checkCollision(Point2f collZoneCorners[4]);
};

class ParkingSpot : public Immovable
{
private:
	bool activeTarget;

	Point2f finalPos;
	Vec2f finalOri;
	vector<CarConfiguration*> prePos;
public:
	inline bool isActiveTarget() { return this->activeTarget; }
	inline Point2f getFinalPos() { return this->finalPos; }
	inline Point2f getFinalOri() { return this->finalOri; }
	inline const vector<CarConfiguration*> getPrePos() { return this->prePos; }

	void activate() { this->activeTarget = true; }
	void deactivate() { this->activeTarget = false; }
	void setPreTargets();
	bool checkCollision(Point2f collZoneCorners[4]);

	ParkingSpot(Map* map, const vector<float>& coords);
	virtual ~ParkingSpot();
	virtual void draw() const;
};

#endif // IMMOVABLE_H