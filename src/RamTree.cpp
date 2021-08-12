#include "RamTree.h"
#include "Map.h"
#include <random>
#include <chrono>
#include "brutil.h"

vector< vector<PathElem>(*)(Point2f, float, float) > RamTreeNode::plans = vector< vector<PathElem>(*)(Point2f, float, float) >({ &planPath1,
																																 &planPath2/*,
																																 &planPath2,
																																 &planPath4,
																																 &planPath5 */});


void RamTreeNode::setSegments(const AbstractTrajectory& traj)
{
	for (vector<AbstractSegment*>::const_iterator it = traj.segments.begin(); it != traj.segments.end(); it++)
		this->segments.push_back((*it)->WTFNewSegment(this->tree->map));
}

RamTreeNode::RamTreeNode(RamTree* tree, const Point2f& pos, const Vec2f ori) : tree(tree),
																				pos(pos),
																				parent(0),
																				childs(),
																				segments(),
																				dist(0),
																				rootDist(0)
{
	this->ori = normalize(ori);
}

RamTreeNode::RamTreeNode(RamTree* tree, RamTreeNode* parent, const AbstractTrajectory& traj) : tree(tree),
																			parent(parent),
																			childs(),
																			segments(),
																			dist(traj.length)
{
	this->rootDist = this->parent->getRootDist() + this->dist;
	this->setSegments(traj);
	int lastSegmentIndex = this->segments.size() - 1;
	this->pos = traj.endPos;
	this->ori = traj.endOri;
}

RamTreeNode::~RamTreeNode()
{
	for (vector<RamTreeNode*>::iterator it = this->childs.begin(); it != this->childs.end(); it++)
	{
		(*it)->parent = this->parent;
		this->parent->addChild(*it);
	}
	for (vector<Segment*>::iterator it = this->segments.begin(); it != this->segments.end(); it++)
	{
		delete *it;
	}
}

vector<PathElem> RamTreeNode::calculateDist(const Point2f& pos, const Vec2f ori)
{
	float theta = getAngleBetween(this->ori, ori);
	Point2f target, preTarget = pos - this->pos;
	float phi = getAngleBetween(this->ori, Vec2f(1, 0));
	target = rotateVector(preTarget, phi);
	vector<vector<PathElem>> paths;
	for (int i = 0; i < RamTreeNode::plans.size(); i++)
	{
		vector<PathElem> tempPathElems = plans[i](target, theta, this->tree->getMinTurnRadius());
		if (tempPathElems.size())
		{
			paths.push_back(tempPathElems);
		}
		
		tempPathElems = plans[i](Point2f(-target.x, target.y), -theta, this->tree->getMinTurnRadius());
		if (tempPathElems.size())
		{
			reverse(tempPathElems);
			paths.push_back(tempPathElems);
		}
		
		tempPathElems = plans[i](Point2f(target.x, -target.y), -theta, this->tree->getMinTurnRadius());
		if (tempPathElems.size())
		{
			mirror(tempPathElems);
			paths.push_back(tempPathElems);
		}
		
		tempPathElems = plans[i](Point2f(-target.x, -target.y), theta, this->tree->getMinTurnRadius());
		if (tempPathElems.size())
		{
			reverse(tempPathElems);
			mirror(tempPathElems);
			paths.push_back(tempPathElems);
		}
	}
	return rscMin(paths);
}

float RamTreeNode::calculateEucledeanDist(const Point2f& pos)
{
	return norm(pos - this->pos);
}

void RamTreeNode::addChild(RamTreeNode* child)
{
	this->childs.push_back(child);
	child->parent = this;
}

void RamTreeNode::draw()
{
	for (vector<Segment*>::iterator it = this->segments.begin(); it != this->segments.end(); it++)
	{
		(*it)->draw();
	}
}

bool RamTree::checkTarget(RamTreeNode* node)
{
	for (vector<CarConfiguration*>::iterator it = this->targets.begin(); it != this->targets.end(); it++)
	{
		if (node->calculateEucledeanDist((*it)->pos) < targetProximity)
		{
			vector<PathElem> shortestRSC = node->calculateDist((*it)->pos, (*it)->ori);
			if (shortestRSC.size())
			{
				bool truncated = false;
				NearestNode nn{ node, shortestRSC, sumRSCPath(shortestRSC) };
				this->targetReached = true;
				RamTreeNode* newNode = 0;
				this->addNode(nn, -1, truncated, newNode, true);
				if (!truncated)
				{
					CarConfiguration* config = *it;
					while (config->parent)
					{
						newNode = this->addFixNode(newNode, config->t);
						config = config->parent;
					}
					return true;
				}
				else
					this->targetReached = false;
			}
		}
	}
	return false;
}

RamTree::RamTree(Map* map, const Point2f& pos, const Vec2f ori, const vector<CarConfiguration*>& targets, float minTurnRadius, float increment, float targetProximity) : map(map),
	                                                                                                                                      targetProximity(targetProximity),
	                                                                                                                                      increment(increment),
																																		  targetReached(false),
																																		  minTurnRadius(minTurnRadius),
																																		  targetNode()
{
	this->targets = targets;
	RamTreeNode* newNode = new RamTreeNode(this, pos, ori);
	this->vertices.push_back(newNode);
}

NearestNode RamTree::findNearestNode(const Point2f& pos, const Vec2f ori)
{
	RamTreeNode* minNode = 0;
	vector<PathElem> shortestRSC;
	for (vector<RamTreeNode*>::iterator it = this->vertices.begin(); it != this->vertices.end(); it++)
	{
		vector<PathElem> currRSC = (*it)->calculateDist(pos, ori);
		if (!currRSC.size())
			continue;
		if (!shortestRSC.size() || rscComp(currRSC, shortestRSC))
		{
			shortestRSC = currRSC;
			minNode = *it;
		}
	}
	return NearestNode{ minNode, shortestRSC, sumRSCPath(shortestRSC) };
}

bool RamTree::addNode(NearestNode& nearestNode, float truncLength, bool& truncated, RamTreeNode*& newNode, bool isTarget)
{
	AbstractTrajectory* traj = this->truncatePath(nearestNode, truncLength, truncated);
	if (!traj)
		return false;
	newNode = new RamTreeNode(this, nearestNode.node, *traj);
	delete traj;
	this->vertices.push_back(newNode);
	if (isTarget && !truncated)
		this->targetNode = newNode;
	if (!this->targetReached && checkTarget(newNode))
		return true;
	return false;
}

RamTreeNode* RamTree::addFixNode(RamTreeNode* nearestNode, AbstractTrajectory* t)
{
	RamTreeNode* newNode = new RamTreeNode(this, nearestNode, *t);
	this->vertices.push_back(newNode);
	this->targetNode = newNode;
	return newNode;
}

AbstractTrajectory* RamTree::truncatePath(NearestNode& nearestNode, float truncLength, bool& truncated, float stepsize)
{
	Point2f segStartPos = nearestNode.node->getPos();
	Vec2f segStartOri = nearestNode.node->getOri();
	AbstractTrajectory* traj = new AbstractTrajectory(segStartPos, segStartOri, stepsize);
	for (vector<PathElem>::const_iterator it = nearestNode.path.begin(); it != nearestNode.path.end(); it++)
	{
		if (it->isCurve)
		{
			traj->addCurveSegment(it->arc * it->isLeft * it->isForward, this->minTurnRadius, it->isLeft == -1);
			segStartPos = traj->getEndPos();
			segStartOri = traj->getEndOri();
		}
		else
		{
			traj->addLinearSegment(it->length, it->isForward == 1);
			segStartPos = traj->getEndPos();
		}
	}
	if (!traj->truncate(this->map, truncLength, truncated, stepsize))
	{
		delete traj;
		return 0;
	}
	return traj;
}

void RamTree::draw()
{
	for (vector<RamTreeNode*>::iterator it = this->vertices.begin(); it != this->vertices.end(); it++)
	{
		(*it)->draw();
	}
}

bool RamTree::addRRTNode()
{
	if (this->vertices.size() == 1)
		if (checkTarget(this->vertices[0]))
			return true;
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distributionX(this->map->getXMin(), this->map->getXMax());
	std::uniform_real_distribution<double> distributionY(this->map->getYMin(), this->map->getYMax());
	std::uniform_real_distribution<double> distributionPhi(0, CV_2PI);
	Point2f randomPoint(distributionX(generator), distributionY(generator));
	float randomPhi = distributionPhi(generator);
	Vec2f randomOri(cos(randomPhi), sin(randomPhi));
	NearestNode nearestNode = this->findNearestNode(randomPoint, randomOri);
	if (!nearestNode.node)
		return false;
	bool truncated = false;
	RamTreeNode* newNode = 0;
	if (this->addNode(nearestNode, this->increment, truncated, newNode))
		return true;
	return false;
}

void  RamTree::growRRT(int steps)
{
	for (int i = 0; i < steps; i++)
	{
		if (this->addRRTNode())
			break;
	}
}

Trajectory* RamTree::composeTrajectoryFromTree()
{
	if (!this->targetReached)
		return 0;
	
	vector<Segment*> reverseSegments;
	RamTreeNode* tempNode = this->targetNode;
	while (tempNode != this->vertices[0])
	{
		for (int i = tempNode->segments.size() - 1;  i >= 0; i--)
		{
			reverseSegments.push_back(tempNode->segments[i]);
		}
		tempNode = tempNode->parent;
	}
	if (!reverseSegments.size())
		return 0;
	Trajectory* t = new Trajectory(this->map, this->vertices[0]->pos, this->vertices[0]->ori);
	while (!reverseSegments.empty())
	{
		Segment* last = reverseSegments[reverseSegments.size()-1];
		if (last->antipattern() == 0)
		{
			LinearSegment* llast = dynamic_cast<LinearSegment*>(last);
			t->addLinearSegment(last->getLength(), llast->getDirection() == 1);
		}
		else
		{
			CurveSegment* clast = dynamic_cast<CurveSegment*>(last);
			t->addCurveSegment(clast->getAngle(), clast->getRadius(), clast->isRight());
		}
		reverseSegments.pop_back();
	}
	return t;
}

RamTree::~RamTree()
{
	for (vector<RamTreeNode*>::iterator it = this->vertices.begin(); it != this->vertices.end(); it++)
	{
		delete *it;
	} 
}
