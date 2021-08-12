#include "Map.h"
#include "MapObject.h"
#include "brutil.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <math.h>

Map::Map(const string& mapFile, const string& windowName, int width, int height, Scalar background) : objects(),
																							          pss(),
                                                                                                      windowName(windowName),
	                                                                                                  map(height, width, CV_32FC3, background),
	                                                                                                  background(background),
																									  isAnimating(false),
	                                                                                                  isAnimationFinished(false),
																							          x_min(0),
																									  x_max(width),
																								      y_min(0),
																									  y_max(height),
	                                                                                                  scale(1),
	                                                                                                  offset_x(0),
	                                                                                                  offset_y(0),
																									  activePSIndex(0),
																									  tree(0)
{
	this->vehicle = new Vehicle(this, this->vPos, this->vOri, this->vLength, this->vWidth, this->vWheelbase, this->vRearOverhang);

	filebuf fb;
	if (fb.open(mapFile, std::ios::in))
	{
		istream is(&fb);
		while (is.good())
		{
			const int max_line_length = 1024;
			char line[max_line_length];
			if (is.getline(line, max_line_length).fail())
				throw runtime_error("Error while reading map file. Possible reason is line length longer than 1024 or empty file.");
			vector<float> tokens;
			const char* delim = " \n";
			char* token = strtok(line, delim);
			int objectType = atoi(token);
			while (token)
			{
				token = strtok(NULL, " \n");
				if (token)
					tokens.push_back((float)atof(token));
			}
			Immovable* object;
			switch (objectType)
			{
			case Immovable::ObjectType::PILLAR:
				object = new Pillar(this, tokens);
				break;
			case Immovable::ObjectType::WALL:
				object = new Wall(this, tokens);
				break;
			case Immovable::ObjectType::PARKING_SPOT:
				object = new ParkingSpot(this, tokens);
				pss.push_back(dynamic_cast<ParkingSpot*>(object));
				break;
			default:
				break;
			}
			objects.push_back(object);
		}
		if (pss.size())
		{
			for (vector<ParkingSpot*>::iterator it = pss.begin(); it != pss.end(); it++)
				(*it)->setPreTargets();
			this->pss[this->activePSIndex]->activate();
			this->createNewTree();
		}
		else
			throw runtime_error("No parking spots were defined in the input file.");


		bool extremesSet = false;
		for (vector<Immovable*>::const_iterator it = this->objects.begin(); it != this->objects.end(); it++)
		{
			for (vector<Point2f>::const_iterator rit = (*it)->points.begin(); rit != (*it)->points.end(); rit++)
			{
				if (!extremesSet)
				{
					this->x_min = (*rit).x;
					this->x_max = (*rit).x;
					this->y_min = (*rit).y;
					this->y_max = (*rit).y;
					extremesSet = true;
				}
				else
				{
					this->x_min = min<float>((*rit).x, this->x_min);
					this->x_max = max<float>((*rit).x, this->x_max);
					this->y_min = min<float>((*rit).y, this->y_min);
					this->y_max = max<float>((*rit).y, this->y_max);
				}
			}
		}
		this->scale = min<float>((float)width / (this->x_max - this->x_min) * 0.9f, (float)height / (this->y_max - this->y_min) * 0.9f);
		this->offset_x = float(width) / 2.0f - (this->x_min + (this->x_max - this->x_min) / 2.0f) * scale;
		this->offset_y = float(height) / 2.0f - (this->y_min + (this->y_max - this->y_min) / 2.0f) * scale;
	}
	else
	{
		throw runtime_error("Could not open mapfile");
	}
	this->calculateCVPoints();

	setMouseCallback(this->windowName, Map::mouseCallback, this);
}

Map::~Map()
{
	for (vector<Immovable*>::const_iterator it = this->objects.begin(); it != this->objects.end(); it++)
	{
		delete *it;
	}
	this->unsetBlob();
	if (this->vehicle)
		delete this->vehicle;
	if (this->tree)
		delete this->tree;
}

void Map::draw()
{
	this->map = this->background;
	for (vector<Immovable*>::const_iterator it = this->objects.begin(); it != this->objects.end(); it++)
	{
		(*it)->draw();
	}
	if (this->tree)
		this->tree->draw();
	if (this->blob)
		this->blob->draw(this->map);
	this->vehicle->draw();
	imshow(this->windowName, this->map);
}

void Map::setBlob(Point2i center, int radius)
{
	this->unsetBlob();
	this->blob = new Blobstacle(center, radius);
	this->blob->calculateRealPoints(this->offset_x, this->offset_y, this->scale);
}

void Map::unsetBlob()
{
	if (this->blob)
		delete this->blob;
	this->blob = 0;
}

bool Map::planTrajectory(int steps)
{
	this->reset();
	for (int i = 0; i < steps; i++)
	{
		if (waitKey(1) == 'p')
		{
			this->reset();
			return true;
		}
		if (this->addRRTNode())
		{
			Trajectory* t = this->composeTrajectoryFromTree();
			t->addLinearSegment(norm(this->pss[this->activePSIndex]->getFinalPos() - t->getEndPos()), false);
			t->setColor(Scalar(0, 0, 1, 1));
			this->vehicle->setTraj(t);
			this->draw();
			return true;
		}
		this->draw();
	}
	return false;
}

void Map::reset()
{
	if (this->vehicle)
		delete this->vehicle;
	this->vehicle = new Vehicle(this, this->vPos, this->vOri, this->vLength, this->vWidth, this->vWheelbase, this->vRearOverhang);
	unsetBlob();
	createNewTree();
	this->isAnimating = false;
	this->isAnimationFinished = false;
}

void Map::createNewTree()
{
	if (this->tree);
		delete this->tree;
	this->tree = new RamTree(this, this->vPos, this->vOri, this->pss[this->activePSIndex]->getPrePos(), this->vehicle->getRearAxleCenterTurnRadius());
}

void Map::startStop()
{
	isAnimating = !isAnimating;
}

void Map::simulateStep()
{
	if (this->isAnimating && !this->isAnimationFinished)
	{
		this->isAnimationFinished = this->vehicle->stepTraj();
	}
}

bool Map::checkCollision(const Point2f& pos, const Vec2f ori, float safety)
{
	Point2f realCollZoneCorners[4];
	const Vec2f* collZoneCorners = this->vehicle->getCollZone(safety);
	float theta = getAngleBetween(Vec2f(1, 0), ori);
	for (int i = 0; i < 4; i++)
	{
		realCollZoneCorners[i] = (Vec2f)pos + rotateVector(collZoneCorners[i], theta);
	}
	delete collZoneCorners;

	Point2f tail = (Vec2f)pos - ori * this->vehicle->getRearOverhang();
	Point2f nose = (Vec2f)tail + ori * this->vehicle->getLenght();
	Point2f center = (tail + nose) / 2;

	float range = sqrt(pow(this->vehicle->getLenght() + this->vehicle->getSafety() * 2, 2) + pow(this->vehicle->getWidth() + this->vehicle->getSafety() * 2, 2));

	for (vector<Immovable*>::const_iterator it = this->objects.begin(); it != this->objects.end(); it++)
	{
		if (norm(center - (*it)->getCentroid()) < range + (*it)->getRange())
		{
			if ((*it)->checkCollision(realCollZoneCorners))
				return true;
		}
	}
	return false;
}

bool Map::addRRTNode()
{
	return tree->addRRTNode();
}

void Map::activateNextSpot(bool forward)
{
	this->reset();
	this->pss[this->activePSIndex]->deactivate();
	int step = 1;
	if (!forward)
		step = pss.size() - 1;
	this->activePSIndex = (this->activePSIndex + step) % pss.size();
	this->pss[this->activePSIndex]->activate();

	this->tree->targets = this->pss[this->activePSIndex]->getPrePos();
}

void Map::mouseCallback(int event, int x, int y, int flags, void* userdata)
{
	Map* map = (Map*)userdata;
	if (map->isAnimating)
		return;
	switch (event)
	{
		case EVENT_LBUTTONUP:
			map->setBlob(Point2i(x, y));
			map->draw();
			break;
		case EVENT_RBUTTONUP:
			map->unsetBlob();
			map->draw();
			break;
	}
}

Trajectory* Map::composeTrajectoryFromTree()
{
	return this->tree->composeTrajectoryFromTree();
}

void Map::calculateCVPoints()
{
	for (vector<Immovable*>::const_iterator it = this->objects.begin(); it != this->objects.end(); it++)
	{
		(*it)->calculateCVPoints();
	}
	this->vehicle->calculateCVPoints();
}
