/*
 * VectorMapDrawObject.h
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */

#ifndef VECTORMAPDRAWOBJECT_H_
#define VECTORMAPDRAWOBJECT_H_


#include "DrawObject.h"
#include "vector_map.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <vector>


using std::vector;


class PointListDrawObject: public DrawObject
{
public:
	PointListDrawObject ();
	PointListDrawObject (vector<Point3>);
	PointListDrawObject (VectorMap &v);
	virtual ~PointListDrawObject();

	void initialize ();
	void draw ();

	inline void add (const Point3 &p)
	{ pointList.push_back(p); }

	inline size_t size () { return pointList.size(); }
	inline void setSize (float s) { pointSize = s; }

	typedef shared_ptr<PointListDrawObject> Ptr;

private:
	vector<Point3> pointList;
	GLuint vbo;
	float pointSize;
};

#endif /* VECTORMAPDRAWOBJECT_H_ */
