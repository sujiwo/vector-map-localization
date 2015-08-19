/*
 * LaneDrawObject.cpp
 *
 *  Created on: Apr 11, 2015
 *      Author: sujiwo
 */

#include <vector>
#include <GL/glew.h>
#include <GL/gl.h>
#include "RectangleDrawObject.h"
#include "SceneManager.h"


using std::vector;


RectangleDrawObject::RectangleDrawObject (const Point3 &p1, const Point3 &p2, double sc) :
	point1(p1), point2(p2),
	scale (sc)
{
	glGenBuffers (1, &vbo);
}


RectangleDrawObject::~RectangleDrawObject()
{
	glDeleteBuffers (1, &vbo);
}


void RectangleDrawObject::createBox (
	const Point2 &source1,
	const Point2 &source2,
	Point2 &result1,
	Point2 &result2,
	Point2 &result3,
	Point2 &result4,
	double scale)
{
	double grad1 = ( source2.y() - source2.y() ) / ( source2.x() - source2.x() ),
		grad2 = 1 / grad1;

	double s = grad2 / sqrt (1 + grad2),
		c = s / grad2;
	double len2 = scale / 2;
	Point2 v (len2 * c, len2 * s);

	result1 = source1 - v,
	result2 = source1 + v,
	result3 = source2 - v,
	result4 = source2 + v;
}


Point2 Point3to2 (const Point3 &p3)
{ return Point2 (p3.x(), p3.y()); }


Point3 Point2to3 (const Point2 &p2, double zval)
{ return Point3 (p2.x(), p2.y(), zval); }


void RectangleDrawObject::createBox (
	const Point3 &source1,
	const Point3 &source2,
	Point3 &result1,
	Point3 &result2,
	Point3 &result3,
	Point3 &result4,
	double scale)
{
	Point2 s1 = Point3to2 (source1),
		s2 = Point3to2 (source2);
	Point2 r1, r2, r3, r4;

	createBox (s1, s2, r1, r2, r3, r4, scale);
	result1 = Point2to3 (r1, s1.z());
	result2 = Point2to3 (r2, s1.z());
	result3 = Point2to3 (r3, s2.z());
	result4 = Point2to3 (r4, s2.z());
}


void RectangleDrawObject::initialize ()
{
	Point3 rect[4];
	createBox (point1, point2, rect[0], rect[1], rect[2], rect[3], scale);

	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	glBufferData (GL_ARRAY_BUFFER, sizeof(Point3)*4, rect, GL_STATIC_DRAW);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}


void RectangleDrawObject::draw()
{
	scene->getShader()->use();
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	//setColor(Vector3(1.0, 1.0, 0));
	doSetColor ();
	glPointSize (4.0);
	glDrawArrays (GL_TRIANGLE_STRIP, 0, 4);
	//glFlush ();

	glDisableClientState (GL_VERTEX_ARRAY);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}
