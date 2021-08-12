#ifndef MAP_H
#define MAP_H

#include "Immovable.h"
#include "Vehicle.h"
#include "Blobstacle.h"
#include "RamTree.h"

using namespace std;
using namespace cv;


class Map
{
private:
	float x_min;
	float x_max;
	float y_min;
	float y_max;

	float offset_x;
	float offset_y;
	float scale;

	int activePSIndex;
	bool isAnimating;
	bool isAnimationFinished;
	Scalar background;

	const Point2f vPos = Point2f(10.78f, 19.06f);
	const Vec2f vOri = Vec2f(-0.1961f, -0.9805f);
	const float vLength = 4.87f;
	const float vWidth = 2.12f;
	const float vWheelbase = 2.85f;
	const float vRearOverhang = 1.07f;

	vector<Immovable*> objects;
	vector<ParkingSpot*> pss;
	Blobstacle* blob = 0;
	Vehicle* vehicle;
	RamTree* tree;

	string windowName;

	void createNewTree();
	void calculateCVPoints();
public:
	Mat map;

	Map(const string& mapFile, const string& windowName, int width, int height, Scalar background = Scalar(0.4, 0.4, 0.4 , 1.0));

	inline float getOffsetX() const { return this->offset_x; }
	inline float getOffsetY() const { return this->offset_y; }
	inline float getScale() const { return this->scale; }
	inline float getXMin() const { return this->x_min; }
	inline float getXMax() const { return this->x_max; }
	inline float getYMin() const { return this->y_min; }
	inline float getYMax() const { return this->y_max; }
	inline const Vehicle& getVehicle() { return *(this->vehicle); }
	virtual ~Map();
	void draw();
	void setBlob(Point2i center, int radius = 20);
	void unsetBlob();

	bool planTrajectory(int steps);
	void reset();
	void startStop();
	void simulateStep();
	bool checkCollision(const Point2f& pos, const Vec2f ori, float safety = -1);
	void activateNextSpot(bool forward = true);
	Trajectory* composeTrajectoryFromTree();

	bool addRRTNode();

	static void mouseCallback(int event, int x, int y, int flags, void* userdata);
};

#endif // !MAP_H