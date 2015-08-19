/*
 * TrafficLights.h
 *
 *  Created on: Apr 13, 2015
 *      Author: sujiwo
 */

#ifndef TRAFFICLIGHTS_H_
#define TRAFFICLIGHTS_H_

#include "DrawObjects.h"
#include <vector>
#include <stdint.h>

#include "vector_map.h"



using std::vector;


const float lampRadius = 0.2;


#define SIGNAL_RED 1
#define SIGNAL_GREEN 2
#define SIGNAL_YELLOW 3


class Camera;


struct TrafficLight {
	Point3 center;
	vector<Point3> points;
	vector<unsigned int> idx;
	unsigned int color;
	unsigned int id;
	Vector3 direction;
};



class TrafficLights: public VectorMapObject
{
public:
	TrafficLights (VectorMap *v, RenderWidget *w);

	void initialize ();

	void draw ();

private:
	VectorMap *vmap;
	vector<Point3> signalCenterPoints;

	vector<Point3> allPts;
	vector<TrafficLight> signalObjects;

	static Vector3 angle2UnitVector (const float Hang, const float Vang);
	static vector<Point3> signalPoint2Circle (const Point3 &centerPoint, float Hang, float Vang);
	static vector<Point3> signalPoint2Circle (VectorMap *map, const Signal &sgn);

};

#endif /* TRAFFICLIGHTS_H_ */
