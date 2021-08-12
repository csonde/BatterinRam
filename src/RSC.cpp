#include "RSC.h"
#include "brutil.h"

float sumRSCPath(const vector<PathElem>& rsc)
{
	float len = 0;
	for (vector<PathElem>::const_iterator it = rsc.begin(); it != rsc.end(); it++)
		len += it->length;
	return len;
}

bool rscComp(const vector<PathElem>& rsc1, const vector<PathElem>& rsc2)
{
	float len1 = 0, len2 = 0;
	for (vector<PathElem>::const_iterator it = rsc1.begin(); it != rsc1.end(); it++)
	{
		len1 += it->length;
	}
	for (vector<PathElem>::const_iterator it = rsc2.begin(); it != rsc2.end(); it++)
	{
		len2 += it->length;
	}
	return len1 < len2;
}

vector<PathElem> rscMin(const vector<vector<PathElem>>& paths)
{
	vector<PathElem> shortest;
	if (!paths.empty())
	{
		shortest = paths[0];
		for (vector<vector<PathElem>>::const_iterator it = paths.begin()+1; it != paths.end(); it++)
		{
			if (!rscComp(shortest, *it))
				shortest = *it;
		}
	}
	return shortest;
}


void reverse(vector<PathElem>& pathElems)
{
	for (vector<PathElem>::iterator it = pathElems.begin(); it != pathElems.end(); it++)
	{
		it->isForward *= -1;
	}
}

void mirror(vector<PathElem>& pathElems)
{
	for (vector<PathElem>::iterator it = pathElems.begin(); it != pathElems.end(); it++)
	{
		it->isLeft *= -1;
	}
}

vector<PathElem> planPath1(Point2f targetPos, float phi, float rMin)
{
	float r, theta;
	getPolar(targetPos, r, theta);
	float t, u, v;
	getPolar(Point2f(targetPos.x - rMin * sin(phi), targetPos.y - rMin* (1 - cos(phi))), u, t);
	v = reduceAngle(phi - t);

	vector<PathElem> ret;
	//if (t >= 0 && u >= 0 && v >= 0)
	if (u >= 0)
	{
		ret.push_back(PathElem{ true, t, rMin * abs(t), 1, 1 });
		ret.push_back(PathElem{ false, 0, u, 0, 1 });
		ret.push_back(PathElem{ true, v, rMin * abs(v), 1, 1 });
	}
	return ret;
}

vector<PathElem> planPath2(Point2f targetPos, float phi, float rMin)
{
	float r, theta;
	getPolar(targetPos, r, theta);
	float t, u, v;
	getPolar(Point2f(targetPos.x + rMin * sin(phi), targetPos.y - rMin * (1 + cos(phi))), u, t);
	vector<PathElem> ret;
	if (u * u >= 4 * rMin * rMin)
	{
		u = sqrt(u * u - 4 * rMin * rMin);
		t = reduceAngle(t + atan2(2 * rMin, u));
		v = reduceAngle(t - phi);
		//if (t >= 0 && u >= 0 && v >= 0)
		if (u >= 0)
		{
			ret.push_back(PathElem{ true, t, rMin * abs(t), 1, 1 });
			ret.push_back(PathElem{ false, 0, u, 0, 1});
			ret.push_back(PathElem{ true, v, rMin * abs(v), -1, 1 });
		}
	}
	return ret;
}

vector<PathElem> planPath3(Point2f targetPos, float phi, float rMin)
{
	float theta;
	float xi = targetPos.x - rMin * sin(phi);
	float eta = targetPos.y - rMin * (1 - cos(phi));
	float t, u, v;
	getPolar(Point2f(xi,eta), u, theta);
		
	vector<PathElem> ret;
	if (u <= 4 * rMin)
	{
		float A = acos(u / (4 * rMin));
		t = reduceAngle(theta + CV_PI /2 + A);
		u = reduceAngle(CV_PI - 2 * A);
		v = reduceAngle(phi - t - u);
		
		//if (t >= 0 && u >= 0 && v >= 0)
		//{
			ret.push_back(PathElem{ true, t, rMin * abs(t), 1, 1 });
			ret.push_back(PathElem{ true, u, rMin * abs(u), -1, -1 });
			ret.push_back(PathElem{ true, v, rMin * abs(v), 1, 1 });
		//}
	}
	return ret;
}

vector<PathElem> planPath4(Point2f targetPos, float phi, float rMin)
{
	float theta;
	float xi = targetPos.x - rMin * sin(phi);
	float eta = targetPos.y - rMin * (1 - cos(phi));
	float t, u, v;
	getPolar(Point2f(xi, eta), u, theta);

	vector<PathElem> ret;
	if (u <= 4 * rMin)
	{
		float A = acos(u / (4 * rMin));
		t = reduceAngle(theta + CV_PI / 2 + A);
		u = reduceAngle(CV_PI - 2 * A);
		v = reduceAngle(t + u - phi);

		//if (t >= 0 && u >= 0 && v >= 0)
		//{
		ret.push_back(PathElem{ true, t, rMin * abs(t), 1, 1 });
		ret.push_back(PathElem{ true, u, rMin * abs(u), -1, -1 });
		ret.push_back(PathElem{ true, v, rMin * abs(v), 1, -1 });
		//}
	}
	return ret;
}

vector<PathElem> planPath5(Point2f targetPos, float phi, float rMin)
{
	float r, theta;
	float xi = targetPos.x - rMin * sin(phi);
	float eta = targetPos.y - rMin * (1 - cos(phi));
	getPolar(Point2f(xi, eta), r, theta);
	float t, u, v;

	vector<PathElem> ret;
	if (r <= 4 * rMin)
	{
		u = acos(1 - r * r / (8 * rMin * rMin));
		float A = asin(2 * rMin * sin(u) / r);
		t = reduceAngle(theta + CV_PI / 2 - A);
		v = reduceAngle(t - u - phi);

		//if (t >= 0 && u >= 0 && v >= 0)
		//{
		ret.push_back(PathElem{ true, t, rMin * abs(t), 1, 1 });
		ret.push_back(PathElem{ true, u, rMin * abs(u), -1, 1 });
		ret.push_back(PathElem{ true, v, rMin * abs(v), 1, -1 });
		//}
	}
	return ret;
}
