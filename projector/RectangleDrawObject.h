/*
 * LaneDrawObject.h
 *
 *  Created on: Apr 11, 2015
 *      Author: sujiwo
 */

#ifndef RECTANGLEDRAWOBJECT_H_
#define RECTANGLEDRAWOBJECT_H_

#include "DrawObject.h"
#include <GL/gl.h>


class RectangleDrawObject: public DrawObject
{
public:
	RectangleDrawObject (const Point3 &p1, const Point3 &p2, double sc);
	virtual ~RectangleDrawObject();

	void initialize ();
	void draw ();

	typedef shared_ptr<RectangleDrawObject> Ptr;

	static void createBox (
		const Point2 &source1,
		const Point2 &source2,
		Point2 &result1,
		Point2 &result2,
		Point2 &result3,
		Point2 &result4,
		double scale);

	static void createBox (
		const Point3 &source1,
		const Point3 &source2,
		Point3 &result1,
		Point3 &result2,
		Point3 &result3,
		Point3 &result4,
		double scale);

private:
	Point3 point1, point2;
	GLuint vbo;
	double scale;
};

#endif /* RECTANGLEDRAWOBJECT_H_ */
