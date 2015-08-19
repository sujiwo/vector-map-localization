/*
 * LineList.cpp
 *
 *  Created on: Jul 19, 2015
 *      Author: sujiwo
 */

#include "LineList.h"
#include "RenderWidget.h"

#define QT_NO_KEYWORDS
#undef signals
#include "vector_map.h"


LineList::LineList (VectorMap *vm, double s, RenderWidget *w) :
	VectorMapObject (w),
	scale(s)
{
	for(std::map<int, Line>::iterator it=vm->lines.begin(); it!=vm->lines.end(); it++) {
		Line l = it->second;
		Point3 p1, p2;
		p1.x() = vm->points[l.bpid].bx;
		p1.y() = vm->points[l.bpid].ly;
		p1.z() = vm->points[l.bpid].h;
		p2.x() = vm->points[l.fpid].bx;
		p2.y() = vm->points[l.fpid].ly;
		p2.z() = vm->points[l.fpid].h;

		Point3 r1, r2, r3, r4;
		LineList::createBox (p1, p2, r1, r2, r3, r4, scale);
		points.push_back(r1);
		points.push_back(r2);
		points.push_back(r3);
		points.push_back(r3);
		points.push_back(r4);
		points.push_back(r2);
	}

	setColor(0, 255, 0);
}


void LineList::initialize()
{
	initBuffer ();
	vbuffer->bind();
	int len = points.size();
	vbuffer->allocate(points.data(), sizeof(Point3)*len);
	vbuffer->release();
}


void LineList::draw()
{
	vbuffer->bind();
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	glwidget->getShader()->setUniformValue("objColor", color2qv(color));
	glDrawArrays (GL_TRIANGLES, 0, points.size());
//	glLineWidth (1.0);
//	glDrawArrays (GL_LINES, 0, points.size());
	//glFlush ();

	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release();
}


Point2 Point3to2 (const Point3 &p3)
{ return Point2 (p3.x(), p3.y()); }


Point3 Point2to3 (const Point2 &p2, double zval)
{ return Point3 (p2.x(), p2.y(), zval); }


void LineList::createBox (
	const Point2 &source1,
	const Point2 &source2,
	Point2 &result1,
	Point2 &result2,
	Point2 &result3,
	Point2 &result4,
	double scale)
{
	Vector2 u = source2 - source1;
	u.normalize();
	u = u * scale/2;
	Point2 s1 = source1 - u,
		s2 = source2 + u;
	Vector2 v = perpendicular (u);
	v.normalize();
	v = v * scale/2;
	result1 = s1 - v;
	result2 = s1 + v;
	result3 = s2 - v;
	result4 = s2 + v;
}


void LineList::createBox (
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
	result1 = Point2to3 (r1, source1.z());
	result2 = Point2to3 (r2, source1.z());
	result3 = Point2to3 (r3, source2.z());
	result4 = Point2to3 (r4, source2.z());
}
