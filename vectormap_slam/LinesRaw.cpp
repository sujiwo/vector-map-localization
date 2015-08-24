/*
 * LinesRaw.cpp
 *
 *  Created on: Aug 21, 2015
 *      Author: sujiwo
 */

#include "LinesRaw.h"
#include "RenderWidget.h"


LinesRaw::LinesRaw (VectorMap *vm, RenderWidget *w) :
	VectorMapObject (w)
{
	for(std::map<int, Line>::iterator it=vm->lines.begin(); it!=vm->lines.end(); it++) {
		Line l = it->second;
		int pid1 = l.bpid, pid2 = l.fpid;
		Point3 p1 = vm->getPoint(pid1),
			p2 = vm->getPoint(pid2);
		points.push_back(p1);
		points.push_back(p2);
	}

	setColor(0, 255, 0);
}


void LinesRaw::initialize()
{
	initBuffer ();
	vbuffer->bind();
	int len = points.size();
	vbuffer->allocate(points.data(), sizeof(Point3)*len);
	vbuffer->release();
}


void LinesRaw::draw()
{
	vbuffer->bind();
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	glLineWidth (1.0);
	glwidget->getShader()->setUniformValue("objColor", color2qv(color));
	glDrawArrays (GL_LINES, 0, points.size());

	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release();
}
