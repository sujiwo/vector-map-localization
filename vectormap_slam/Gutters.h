/*
 * Gutters.h
 *
 *  Created on: Aug 5, 2015
 *      Author: sujiwo
 */

#ifndef GUTTERS_H_
#define GUTTERS_H_

#include "AreaList.h"

class Gutters: public VectorMapObject
{
public:
	Gutters(VectorMap *v, RenderWidget *w);
	void initialize ();
	void draw ();

protected:
	VectorMap *map;
	vector <Point3> pointBuffer;
	vector <vector <uint32_t> > pointIndex;
	vector <int> areaIds;
};

#endif /* GUTTERS_H_ */
