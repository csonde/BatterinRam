#include "Blobstacle.h"
#include <opencv2/imgproc.hpp>

Blobstacle::Blobstacle(Point2i center, int radius) : cvCenter(center), cvRadius(radius)
{
}

void Blobstacle::calculateRealPoints(float offset_x, float offset_y, float scale)
{
	center.x = ((float)cvCenter.x - offset_x) / scale;
	center.y = ((float)cvCenter.y - offset_y) / scale;
	radius = (float)cvRadius / scale;
}

void Blobstacle::draw(Mat& map) const
{
	circle(map, cvCenter, cvRadius, Scalar(0, 0, 1, 1), FILLED);
}
