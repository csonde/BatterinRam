#include "Vehicle.h"
#include "Map.h"
#include "brutil.h"
#include <opencv2/imgproc.hpp>

void Vehicle::calculateCVPoints()
{
	for (int i = 0; i < 4; ++i)
	{
		this->cvPoints[i] = Point2i((int)(this->corners[i].x * this->map->getScale() + this->map->getOffsetX()), (int)(this->corners[i].y * this->map->getScale() + this->map->getOffsetY()));
	}
	if (traj)
		traj->calculateCVPoints();
}

void Vehicle::calculateCorners()
{
	Point2f tail = (Vec2f)pos - this->ori * this->rearOverhang;
	Point2f nose = (Vec2f)tail + this->ori * this->length;
	Vec2f oriNorm;
	oriNorm[0] = -this->ori[1];
	oriNorm[1] = this->ori[0];

	this->corners[0] = (Vec2f)tail + oriNorm * this->width / 2;
	this->corners[1] = (Vec2f)tail - oriNorm * this->width / 2;
	this->corners[2] = (Vec2f)nose - oriNorm * this->width / 2;
	this->corners[3] = (Vec2f)nose + oriNorm * this->width / 2;
}

Vehicle::Vehicle(Map* map, const Point2f& pos, const Vec2f& ori, float length, float width, float wheelbase, float rearOverhang, float turnRadius) : MapObject(map),
																																					 pos(pos),
																																					 length(length),
																																					 width(width),
																																					 wheelbase(wheelbase),
																																					 rearOverhang(rearOverhang),
																																					 turnRadius(turnRadius)
{
	this->rearAxleCenterTurnRadius = turnRadius * cos(asin((this->length - this->rearOverhang) / turnRadius)) - width / 2.0f;
	this->ori = normalize(ori);
	this->theta = getAngleBetween(Vec2f(1, 0), this->ori);
	this->calculateCorners();
	this->map = map;
	this->cvPointCount = 4;
	this->cvPoints = new Point2i[this->cvPointCount];
	this->calculateCVPoints();
}

const Vec2f* Vehicle::getCollZone(float customSafety) const
{
	if (customSafety < 0)
		customSafety = this->safety;
	Vec2f oriNorm;
	oriNorm[0] = -this->ori[1];
	oriNorm[1] = this->ori[0];
	Vec2f* collZoneCorners = new Vec2f[4];
	collZoneCorners[0] = (Vec2f)this->corners[0] + customSafety * (-this->ori + oriNorm) - (Vec2f)this->pos;
	collZoneCorners[0] = rotateVector(collZoneCorners[0], -this->theta);
	collZoneCorners[1] = (Vec2f)this->corners[1] + customSafety * (-this->ori - oriNorm) - (Vec2f)this->pos;
	collZoneCorners[1] = rotateVector(collZoneCorners[1], -this->theta);
	collZoneCorners[2] = (Vec2f)this->corners[2] + customSafety * (this->ori - oriNorm) - (Vec2f)this->pos;
	collZoneCorners[2] = rotateVector(collZoneCorners[2], -this->theta);
	collZoneCorners[3] = (Vec2f)this->corners[3] + customSafety * (this->ori + oriNorm) - (Vec2f)this->pos;
	collZoneCorners[3] = rotateVector(collZoneCorners[3], -this->theta);
	return collZoneCorners;
}

Vehicle::~Vehicle()
{
	if (this->traj)
		delete this->traj;
}

void Vehicle::draw() const
{
	const Point2i* pts = this->cvPoints;
	int npt = this->cvPointCount;
	fillPoly(this->map->map, &pts, &npt, 1, Scalar(0, 0.5, 1, 1));
	if (traj)
		traj->draw();
}

void Vehicle::setTraj(Trajectory* newTraj)
{
	if (this->traj)
		delete this->traj;
	this->traj = newTraj;
}

bool Vehicle::stepTraj()
{
	if (!this->traj)
		return true;

	bool finished = traj->step();
	this->moveToTrajPoint(traj->getCurrPos(), traj->getCurrOri());
	this->calculateCVPoints();
	return finished;
}

void Vehicle::moveToTrajPoint(const Point2f& newPos, const Point2f& newOri)
{
	this->pos = newPos;
	this->ori = newOri;
	this->theta = getAngleBetween(Vec2f(1, 0), this->ori);
	this->calculateCorners();
	this->calculateCVPoints();
}

void Vehicle::teleport(const Point2f& newPos, const Point2f& newOri)
{
	if (this->traj)
		delete this->traj;
	this->pos = newPos;
	this->ori = newOri;
	this->theta = getAngleBetween(Vec2f(1, 0), this->ori);
	this->calculateCorners();
	this->calculateCVPoints();
}

