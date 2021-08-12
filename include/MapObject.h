#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include <opencv2/core/mat.hpp>

using namespace cv;

class Map;

class MapObject
{
private:
protected:
	Point2i* cvPoints = 0;
	int cvPointCount = 0;
	Map* map;

	virtual void calculateCVPoints() = 0;
	MapObject() {};
	MapObject(Map* map, int cvPointCount = 0) { this->map = map;  this->cvPointCount = cvPointCount;  if (this->cvPointCount) this->cvPoints = new Point2i[cvPointCount]; }
public:
	virtual void draw() const = 0;

	virtual ~MapObject() { if (this->cvPoints) delete[] this->cvPoints; map = 0; };
	friend class Map;
};

#endif // MAPOBJECT_H