/*
 * LineList.h
 *
 *  Created on: Jul 19, 2015
 *      Author: sujiwo
 */

#ifndef VECTORMAP_SLAM_LINELIST_H_
#define VECTORMAP_SLAM_LINELIST_H_

#include "DrawObjects.h"
#include "Math.h"
#include <vector>


using std::vector;
class VectorMap;


class LineList: public VectorMapObject
{
public:
	LineList (VectorMap *v, double scale, RenderWidget *w);

	void initialize ();
	void draw ();

	static void createBox (
		const Point3 &source1,
		const Point3 &source2,
		Point3 &result1,
		Point3 &result2,
		Point3 &result3,
		Point3 &result4,
		double scale);

	static void createBox (
		const Point2 &source1,
		const Point2 &source2,
		Point2 &result1,
		Point2 &result2,
		Point2 &result3,
		Point2 &result4,
		double scale);


protected:
	vector<Point3> points;
	double scale;
};

#endif /* VECTORMAP_SLAM_LINELIST_H_ */
