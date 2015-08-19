/*
 * VectorMapObjects.cpp
 *
 *  Created on: Jul 8, 2015
 *      Author: sujiwo
 */

#include "VectorMapObjects.h"
#include "VectorMapRenderWidget.h"
#include "debug.h"
#include <GL/glext.h>


VectorMapObject::VectorMapObject (VectorMapRenderWidget *w) :
	glwidget(w)
{}


void VectorMapObject::initBuffer()
{
	vbuffer = new QGLBuffer (QGLBuffer::VertexBuffer);
	vbuffer->bind();
	vbuffer->setUsagePattern(QGLBuffer::StaticDraw);
	bool t = vbuffer->create();
	if (!t)
		debug ("Unable to create vertex buffer");
	vbuffer->release();
}


VectorMapObject::~VectorMapObject()
{
	vbuffer->destroy();
	delete (vbuffer);
}


void PointList::initialize()
{
	initBuffer ();
	vbuffer->bind();
	int len = pointsrc.size();
	vbuffer->allocate(pointsrc.data(), sizeof(Point3)*len);
	//glBufferData (GL_ARRAY_BUFFER, sizeof(Point3)*len, pointsrc.data(), GL_STATIC_DRAW);
	vbuffer->release();
}


void PointList::draw()
{
	vbuffer->bind();
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	glwidget->getShader()->setUniformValue("objColor", color2qv(color));
	glPointSize (size);
	glDrawArrays (GL_POINTS, 0, pointsrc.size());

	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release();
}


TRectangle::TRectangle (
		const Point3 &p1,
		const Point3 &p2,
		const Point3 &p3,
		const Point3 &p4,
		VectorMapRenderWidget *w) :
	VectorMapObject (w)
{
	points[0] = p1;
	points[1] = p2;
	points[2] = p3;
	points[3] = p4;
	color = Vector4 (1.0, 1.0, 1.0, 1.0);
}


void TRectangle::initialize()
{
	initBuffer ();
	vbuffer->bind();

	const int sz = sizeof(points);
	vbuffer->allocate(points, sz);
	vbuffer->release();
}


void TRectangle::draw()
{
	vbuffer->bind ();
	glwidget->getShader()->setUniformValue("objColor", color2qv(color));
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);
	glDrawArrays (GL_TRIANGLE_FAN, 0, 4);
	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release ();
}



#define QT_NO_KEYWORDS
#undef signals
#include "vector_map.h"


MapPoints::MapPoints (const string &path, VectorMapRenderWidget *w) :
	PointList(w)
{
	map = new VectorMap ();
	map->loadAll (path);

	for (int i=1; i<=map->points.size(); i++) {
		Point3 p3 = map->getPoint(i);
		pointsrc.push_back (p3);
	}
}


MapPoints::~MapPoints ()
{
	delete (map);
}
