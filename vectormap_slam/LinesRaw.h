/*
 * LinesRaw.h
 *
 *  Created on: Aug 21, 2015
 *      Author: sujiwo
 */

#ifndef _LINESRAW_H_
#define _LINESRAW_H_

#include "DrawObjects.h"
#include <vector>
#include "Math.h"
#include "vector_map.h"



class LinesRaw: public VectorMapObject {
public:
	LinesRaw (VectorMap *v, RenderWidget *w);

	void initialize ();
	void draw ();

protected:
	vector<Point3> points;

};

#endif /* _LINESRAW_H_ */
