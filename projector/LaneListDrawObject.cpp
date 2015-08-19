/*
 * LaneListDrawObject.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: sujiwo
 */

#include "LaneListDrawObject.h"


void LaneListDrawObject::initialize ()
{
	for(std::map<int, Lane>::iterator it=vmap.lanes.begin();it!=vmap.lanes.end();it++) {
		Lane l=it->second;
		Point3 p1, p2;
		p1.x() = vmap.points[l.bnid].bx;
		p1.y() = vmap.points[l.bnid].ly;
		p1.z() = vmap.points[l.bnid].h;

		//jct.points.push_back(p);
		p2.x() = vmap.points[l.fnid].bx;
		p2.y() = vmap.points[l.fnid].ly;
		p2.z() = vmap.points[l.fnid].h;
		//jct.points.push_back(p);

		Point3 r1, r2, r3, r4;
		LineListDrawObject::createBox (p1, p2, r1, r2, r3, r4, scale);
		points.push_back(r1);
		points.push_back(r2);
		points.push_back(r3);
		points.push_back(r3);
		points.push_back(r4);
		points.push_back(r2);
	}

	setColor (0, 0, 255);
	initBuffer ();
}
