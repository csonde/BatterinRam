#ifndef VEHICLE_H
#define VEHICLE_H

#include <opencv2/core/mat.hpp>
#include <vector>
#include "MapObject.h"
#include "Trajectory.h"


using namespace cv;
using namespace std;

class Vehicle : MapObject
{
private:
	Point2f pos;
	Vec2f ori;
	float theta;

	float length;
	float width;
	float wheelbase;
	float rearOverhang;
	float turnRadius;
	float rearAxleCenterTurnRadius;

	Point2f corners[4];

	float safety = 0.1;

	Trajectory* traj = 0;

protected:
	virtual void calculateCVPoints();
	virtual void calculateCorners();
	Vehicle(Map* map,
		const Point2f& pos = Point2f(10.78f, 19.06f),
		const Vec2f& ori = Vec2f(-0.1961f, -0.9805f),
		float length = 4.87f,
		float width = 2.12f,
		float wheelbase = 2.85f,
		float rearOverhang = 1.07f,
		float turnRadius = 5.5f);
	virtual void moveToTrajPoint(const Point2f& newPos, const Point2f& newOri);
public:
	inline Point2f getPos() const { return this->pos; }
	inline Vec2f getOri() const { return this->ori; }
	inline float getTheta() const { return this->theta; }
	inline float getSafety() const { return this->safety; }
	inline float getLenght() const { return this->length; }
	inline float getWidth() const { return this->width; }
	inline float getWheelBase() const { return this->wheelbase; }
	inline float getRearOverhang() const { return this->rearOverhang; }
	inline float getRearAxleCenterTurnRadius() const { return this->rearAxleCenterTurnRadius; }

	virtual ~Vehicle();

	virtual void draw() const;
	virtual void setTraj(Trajectory* newTraj);

	virtual bool stepTraj();
	virtual void teleport(const Point2f& newPos, const Point2f& newOri);
	const Vec2f* getCollZone(float customSafety = -1) const;

	friend class Map;
};

#endif // VEHICLE_H
