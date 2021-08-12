#ifndef BLOBSTACLE_H
#define BLOBSTACLE_H

#include <opencv2/core/mat.hpp>

using namespace cv;

class Blobstacle
{
private:
	Point2i cvCenter;
	int cvRadius;

	Point2f center;
	float radius;
public:
	Blobstacle(Point2i center, int radius);

	virtual void calculateRealPoints(float offset_x, float offset_y, float scale);
	virtual void draw(Mat& map) const;

	virtual ~Blobstacle() {};
};

#endif // BLOBSTACLE_H