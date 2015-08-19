/*
 * AreaList.h
 *
 *  Created on: Aug 5, 2015
 *      Author: sujiwo
 */

#ifndef _AREALIST_H_
#define _AREALIST_H_

#include "DrawObjects.h"
#include "vector_map.h"
#include "Tesselator.h"





class AreaList: public VectorMapObject {
public:
	AreaList(VectorMap *v, RenderWidget *w);

	void initialize ();
	void draw ();

protected:
	VectorMap *map;
	vector <Point3> pointBuffer;
	vector <uint32_t> firstPointIndices;
	vector <uint32_t> polygonPointCounters;
	vector <GLenum> polygonTypes;
	virtual void loadArea ();
	Tesselator tesselator;
};

#endif /* _AREALIST_H_ */
