#ifndef RSC_H
#define RSC_H

#include <vector>
#include <opencv2/core/mat.hpp>

using namespace std;
using namespace cv;

class RamTreeNode;

struct PathElem
{
	bool isCurve = false;
	float arc = 0;
	float length = 1;
	int isLeft = 1;
	int isForward = 1;
};

struct NearestNode
{
	RamTreeNode* node;
	vector<PathElem> path;
	float totalLenght;
};

float sumRSCPath(const vector<PathElem>& rsc);
bool rscComp(const vector<PathElem>& rsc1, const vector<PathElem>& rsc2);
vector<PathElem> rscMin(const vector<vector<PathElem>>& paths);

void reverse(vector<PathElem>& pathElems);
void mirror(vector<PathElem>& pathElems);

vector<PathElem> planPath1(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath2(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath3(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath4(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath5(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath6(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath7(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath8(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath9(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath10(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath11(Point2f targetPos, float phi, float rMin);
vector<PathElem> planPath12(Point2f targetPos, float phi, float rMin);

#endif RSC_H