#include "Immovable.h"
#include "Map.h"
#include "brutil.h"
#include <iostream>
#include <opencv2/imgproc.hpp>


Immovable::Immovable(Map* map, const vector<float>& coords, int rowNum) : MapObject(map, rowNum), centroid(0,0), range(0)
{
	for (int i = 0; i < rowNum; i++)
	{
		Point2f point;
		point.x = coords[i * 3];
		point.y = coords[i * 3 + 1];
		this->points.push_back(point);
	}
	for (int i = 0; i < this->points.size(); i++)
	{
		centroid += points[i];
	}
	centroid.x /= this->points.size();
	centroid.y /= this->points.size();
	for (int i = 0; i < this->points.size(); i++)
	{
		range = max<float>(range, norm(points[i] - centroid));
	}
}

Immovable::~Immovable()
{
}

bool Immovable::checkCollision(Point2f collZoneCorners[4])
{
	return checkConcavePolyPolyCollision(&(this->points[0]), collZoneCorners);
}

void Immovable::calculateCVPoints()
{
	this->cvPointCount = (int)this->points.size();
	for (int i = 0; i < this->points.size(); ++i)
	{
		this->cvPoints[i] = Point2i((int)(this->points[i].x * this->map->getScale() + this->map->getOffsetX()), (int)(this->points[i].y * this->map->getScale() + this->map->getOffsetY()));
	}
}


Pillar::Pillar(Map* map, const vector<float>& coords) : Immovable(map, coords, 4)
{
}

void Pillar::draw() const
{
	const Point2i* pts = this->cvPoints;
	int npt = this->cvPointCount;
	fillPoly(this->map->map, &pts, &npt, 1, Scalar(0, 0, 1, 1));
}

Wall::Wall(Map* map, const vector<float>& coords) : Immovable(map, coords, 2)
{
}

void Wall::draw() const
{
	line(this->map->map, cvPoints[0], cvPoints[1], Scalar(1, 0, 0, 1), 1);
}

bool Wall::checkCollision(Point2f collZoneCorners[4])
{
	return checkLineConcavePolyCollision(this->points[0], this->points[1], collZoneCorners);
}


void ParkingSpot::setPreTargets()
{
	this->activeTarget = true;
	Point2f frontCenter = (this->points[0] + this->points[3]) / 2;
	Point2f rearCenter = (this->points[1] + this->points[2]) / 2;

	float rearOverhang = this->map->getVehicle().getRearOverhang();
	float safety = this->map->getVehicle().getSafety();
	float sLength = 0.01;
	this->prePos.push_back(new CarConfiguration{ 0, (Vec2f)frontCenter + this->finalOri * 1.1, this->finalOri, 0 });

	int sign[] = { 1, -1 };
	for (int i = 0; i < 2; i++)
	{
		AbstractTrajectory t(this->prePos[0]->pos, this->prePos[0]->ori, sLength);
		t.addCurveSegment(sign[i] * CV_PI / 2, map->getVehicle().getRearAxleCenterTurnRadius(), i);
		bool truncated = false;
		bool valid = t.truncate(this->map, -1, truncated, sLength, true);
		float angle = t.getLength() / map->getVehicle().getRearAxleCenterTurnRadius();
		if (valid && t.getLength() > map->getVehicle().getSafety())
		{
			AbstractTrajectory* backupT = new AbstractTrajectory(t.getEndPos(), t.getEndOri());
			backupT->addCurveSegment(-sign[i] * angle, map->getVehicle().getRearAxleCenterTurnRadius(), i);
			this->prePos.push_back(new CarConfiguration{ this->prePos[0], t.getEndPos(), t.getEndOri(), backupT });
		}
		if (truncated)
		{
			t = AbstractTrajectory(this->prePos[prePos.size() - 1]->pos, this->prePos[prePos.size() - 1]->ori, sLength);
			t.addCurveSegment(sign[i] * (CV_PI / 2 - angle), map->getVehicle().getRearAxleCenterTurnRadius(), ~i);
			valid = t.truncate(this->map, -1, truncated, sLength, true);
			angle = t.getLength() / map->getVehicle().getRearAxleCenterTurnRadius();
			if (valid && t.getLength() > map->getVehicle().getSafety())
			{
				AbstractTrajectory* backupT = new AbstractTrajectory(t.getEndPos(), t.getEndOri());
				backupT->addCurveSegment(-sign[i] * angle, map->getVehicle().getRearAxleCenterTurnRadius(), ~i);
				this->prePos.push_back(new CarConfiguration{ this->prePos[prePos.size() - 1], t.getEndPos(), t.getEndOri(), backupT });
			}
		}
	}
	this->activeTarget = false;
}

ParkingSpot::ParkingSpot(Map* map, const vector<float>& coords) : Immovable(map, coords, 4), activeTarget(false), prePos()
{
	Point2f frontCenter = (this->points[0] + this->points[3]) / 2;
	Point2f rearCenter = (this->points[1] + this->points[2]) / 2;
	Vec2f spotAxis = frontCenter - rearCenter;

	this->finalOri = normalize(spotAxis);
	float space = norm(spotAxis) - this->map->getVehicle().getLenght();
	float rearOverhang = this->map->getVehicle().getRearOverhang();
	this->finalPos = (Vec2f)rearCenter + (space / 2 + rearOverhang) * this->finalOri;
}

ParkingSpot::~ParkingSpot()
{
	for (vector<CarConfiguration*>::iterator it = prePos.begin(); it != prePos.end(); it++)
	{
		if ((*it)->t)
			delete (*it)->t;
		delete (*it);
	}
}



void ParkingSpot::draw() const
{
	const Point2i* pts = this->cvPoints;
	int npt = this->cvPointCount;
	Scalar fillColor(0, 0.5, 0, 1);
	if (activeTarget)
		fillColor = Scalar(0, 1, 0, 1);
	fillPoly(this->map->map, &pts, &npt, 1, fillColor);
	polylines(this->map->map, &pts, &npt, 1, false, Scalar(0, 1, 1, 1), 1);
	if (this->activeTarget)
		for (vector<CarConfiguration*>::const_iterator it = this->prePos.begin(); it != this->prePos.end(); it++)
			circle(this->map->map, Point2f((*it)->pos.x * this->map->getScale() + this->map->getOffsetX(), (*it)->pos.y * this->map->getScale() + this->map->getOffsetY()), 5, Scalar(1, 0, 0, 1), -1);
}


bool ParkingSpot::checkCollision(Point2f collZoneCorners[4])
{
	if (this->activeTarget)
		return false;
	return Immovable::checkCollision(collZoneCorners);
}