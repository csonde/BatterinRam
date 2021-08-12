#ifndef BRUTIL_H
#define BRUTIL_H

#include <opencv2/core/mat.hpp>

using namespace std;
using namespace cv;

bool onSegment(Point p, Point q, Point r);

int orientation(Point2f p, Point2f q, Point2f r);

bool doIntersect(Point2f p1, Point2f q1, Point2f p2, Point2f q2);

void getPolar(const Point2f& xy, float& r, float& theta);

float reduceAngle(const float& phi);

float getAngleBetween(Vec2f base, Vec2f target);

Vec2f rotateVector(const Vec2f& v, float angle);

bool checkPointConcavePolyCollision(const Point2f& p, const Point2f* poly, int polySize = 4);

bool checkLineConcavePolyCollision(const Point2f& lp1, const Point2f& lp2, const Point2f* poly, int polySize = 4);

bool checkConcavePolyPolyCollision(const Point2f* poly1, const Point2f* poly2, int pSize1 = 4, int pSize2 = 4);

#endif BRUTIL_H