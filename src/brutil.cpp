#include "brutil.h"

bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;

    return false;
}

int orientation(Point2f p, Point2f q, Point2f r)
{
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (abs(val) < 1e-10) return 0;
    return (val > 0) ? 1 : 2;
}


bool doIntersect(Point2f p1, Point2f q1, Point2f p2, Point2f q2)
{
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;


    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
}

void getPolar(const Point2f& xy, float& r, float& theta)
{
    r = norm(xy);
    theta = atan2(xy.y, xy.x);
}

float reduceAngle(const float& phi)
{
    float ret = phi;
    while (ret < -CV_PI)
        ret += CV_2PI;
    while (ret > CV_PI)
        ret -= CV_2PI;
    return ret;
}

float getAngleBetween(Vec2f base, Vec2f target)
{
    float cosTheta = target.dot(base) / (norm(target) * norm(base));
    if (cosTheta > 1.0)
        cosTheta = 1.0;
    else if (cosTheta < -1.0)
        cosTheta = -1.0;
    Vec2f baseNorm(-base[1], base[0]);
    if (target.dot(baseNorm) >= 0)
        return acos(cosTheta);
    else
        return CV_2PI - acos(cosTheta);
}

Vec2f rotateVector(const Vec2f& v, float angle)
{
    Vec2f ret;
    ret[0] = v[0] * cos(angle) - v[1] * sin(angle);
    ret[1] = v[0] * sin(angle) + v[1] * cos(angle);
    return ret;
}

bool checkPointConcavePolyCollision(const Point2f& p, const Point2f* poly, int polySize)
{
    int prevOri = 0;
    int side = 0;
    for (int i = 0; i < polySize; i++)
    {
        int ori = orientation(poly[i], poly[(i + 1) % polySize], p);
        if (side == 0)
            side = ori;
        if (i > 0)
        {
            if (ori + prevOri == 0)
                return true;
            else if (ori + side == 3)
            {
                return false;
            }
        }
        prevOri = ori;
    }
    return true;
}

bool checkLineConcavePolyCollision(const Point2f& lp1, const Point2f& lp2, const Point2f* poly, int polySize)
{
    if (checkPointConcavePolyCollision(lp1, poly, polySize))
        return true;
    if (checkPointConcavePolyCollision(lp2, poly, polySize))
        return true;

    for (int i = 0; i < polySize; i++)
        if (doIntersect(lp1, lp2, poly[i], poly[(i + 1) % polySize]))
            return true;

    return false;
}

bool checkConcavePolyPolyCollision(const Point2f* poly1, const Point2f* poly2, int pSize1, int pSize2)
{
    for (int i = 0; i < pSize1; i++)
    if (checkPointConcavePolyCollision(poly1[i], poly2, pSize2))
        return true;
    
    for (int i = 0; i < pSize1; i++)
        for (int j = 0; j < pSize2; j++)
            if (doIntersect(poly1[i], poly1[(i+1)% pSize1], poly2[j], poly2[(j + 1) % pSize2]))
                return true;

    return false;
}

