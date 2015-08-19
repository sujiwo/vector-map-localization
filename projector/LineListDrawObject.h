/*
 * LineListDrawObject.h
 *
 *  Created on: Apr 14, 2015
 *      Author: sujiwo
 */

#ifndef LINELISTDRAWOBJECT_H_
#define LINELISTDRAWOBJECT_H_

#include "DrawObject.h"
#include "vector_map.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <vector>


using std::vector;


class LineListDrawObject: public DrawObject
{
public:
	LineListDrawObject (vector<Point3> &src, double scale);
	LineListDrawObject (VectorMap &v, double scale);
	virtual ~LineListDrawObject();

	void initialize ();
	void draw ();

	typedef shared_ptr<LineListDrawObject> Ptr;

protected:
	void initBuffer ();

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

	VectorMap &vmap;
	vector<Point3> points;
	GLuint vbo;
	double scale;

};

#endif /* LINELISTDRAWOBJECT_H_ */
