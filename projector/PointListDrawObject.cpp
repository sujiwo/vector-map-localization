/*
 * VectorMapDrawObject.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */

#include "PointListDrawObject.h"
#include "SceneManager.h"


PointListDrawObject::PointListDrawObject ()
{
	glGenBuffers (1, &vbo);
}


PointListDrawObject::PointListDrawObject (vector<Point3> lst) :
	pointList (lst),
	pointSize (1.5)
{
	PointListDrawObject ();
}


PointListDrawObject::PointListDrawObject (VectorMap &vmap) :
	pointSize (1.5)
{
	PointListDrawObject ();

	for (int i=1; i<=vmap.points.size(); i++) {
		Point3 p3 = vmap.getPoint(i);
		pointList.push_back (p3);
	}
}


PointListDrawObject::~PointListDrawObject ()
{
	glDeleteBuffers (1, &vbo);
}


void PointListDrawObject::initialize ()
{
	glBindBuffer (GL_ARRAY_BUFFER, vbo);

	int ptlen = pointList.size();

	// watch out for mismatch size (3 or 4 ? not sure)
	glBufferData (GL_ARRAY_BUFFER, sizeof(Point3)*ptlen, pointList.data(), GL_STATIC_DRAW);

	glBindBuffer (GL_ARRAY_BUFFER, 0);
}


void PointListDrawObject::draw()
{
	scene->getShader()->use();
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	doSetColor ();
	glPointSize (pointSize);
	glDrawArrays (GL_POINTS, 0, pointList.size());

	glDisableClientState (GL_VERTEX_ARRAY);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}
