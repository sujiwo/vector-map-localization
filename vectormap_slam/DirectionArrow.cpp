/*
 * DirectionArrow.cpp
 *
 *  Created on: Jul 25, 2015
 *      Author: sujiwo
 */

#include "DirectionArrow.h"
#include "RenderWidget.h"
#include <string>
#include <QtOpenGL/QtOpenGL>



#define arrowSize 2
Point3 Arrows[arrowSize];


DirectionArrow::DirectionArrow (RenderWidget *w) :
	VectorMapObject (w)
{
	setColor (255, 0, 0);
}


void DirectionArrow::initialize()
{
	initBuffer ();
	vbuffer->bind();
	vbuffer->allocate(sizeof(Arrows));
	vbuffer->release();
}


void DirectionArrow::draw()
{
	vbuffer->bind();

	Arrows[0] = glwidget->pose()->transformCameraToWorld(Point3 (0, 1, 3));
	Arrows[1] = glwidget->pose()->transformCameraToWorld(Point3 (0, 1, 10));


	vbuffer->write(0, Arrows, sizeof(Arrows));

	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	//glLineWidth (6);
	glPointSize (6);
	glwidget->getShader()->setUniformValue("objColor", color2qv(color));
	glDrawArrays (GL_POINTS, 0, arrowSize);
	//glFlush ();

	glDisableClientState (GL_VERTEX_ARRAY);

	vbuffer->release();
}
