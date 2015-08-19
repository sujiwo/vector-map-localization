/*
 * PolesDraw.h
 *
 *  Created on: Jul 25, 2015
 *      Author: sujiwo
 */

#ifndef VECTORMAP_SLAM_POLESDRAW_H_
#define VECTORMAP_SLAM_POLESDRAW_H_

#include <DrawObjects.h>
#include <vector>


using std::vector;


class PolesDraw: public VectorMapObject
{
public:
	PolesDraw(VectorMap *v, RenderWidget *w);

	void initialize ();
	void draw ();

protected:
	vector<Point3> polePoints;
};

#endif /* VECTORMAP_SLAM_POLESDRAW_H_ */
