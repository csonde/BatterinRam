#ifndef RAMTREE_H
#define RAMTREE_H

#include <opencv2/core/mat.hpp>
#include <vector>
#include "Trajectory.h"
#include "RSC.h"
#include "Immovable.h"

using namespace std;
using namespace cv;

class RamTree;


class RamTreeNode
{
private:
	RamTree* tree;

	Point2f pos;
	Vec2f ori;

	vector<Segment*> segments;
	float dist;

	float rootDist;

	void setSegments(const AbstractTrajectory& traj);
public:
	static vector< vector<PathElem>(*)(Point2f, float, float) > plans;

	RamTreeNode* parent;
	vector<RamTreeNode*> childs;

	RamTreeNode(RamTree* tree, const Point2f& pos, const Vec2f ori);
	RamTreeNode(RamTree* tree, RamTreeNode* parent, const AbstractTrajectory& nearestNode);
	virtual ~RamTreeNode();

	inline Point2f getPos() const { return this->pos; };
	inline Vec2f getOri() const { return this->ori; };
	inline float getRootDist() const { return this->rootDist; };
	
	vector<PathElem> calculateDist(const Point2f& pos, const Vec2f ori);
	float calculateEucledeanDist(const Point2f& pos);
	void addChild(RamTreeNode* child);
	void draw();

	friend class RamTree;
};

class RamTree
{
private:
	float targetProximity;
	bool targetReached;
	float increment;
	float minTurnRadius;


	vector<RamTreeNode*> vertices;
	RamTreeNode* targetNode;
	bool checkTarget(RamTreeNode* node);
	AbstractTrajectory* truncatePath(NearestNode& nearestNode, float truncLength, bool& truncated, float stepSize = 0.1);
public:
	vector<CarConfiguration*> targets;

	Map* map;

	RamTree(Map* map, const Point2f& pos, const Vec2f ori, const vector<CarConfiguration*>& targets, float minTurnRadius = 3, float increment = 3, float targetProximity = 8);

	NearestNode findNearestNode(const Point2f& pos, const Vec2f ori);
	inline float getMinTurnRadius() { return this->minTurnRadius; }
	bool addNode(NearestNode& nearestNode, float truncLength, bool& truncated, RamTreeNode*& newNode, bool isTarget = false);
	RamTreeNode* addFixNode(RamTreeNode* nearestNode, AbstractTrajectory* t);
	void draw();
	bool addRRTNode();
	void growRRT(int steps);
	Trajectory* composeTrajectoryFromTree();
	virtual ~RamTree();
};

#endif // RAMTREE_H

