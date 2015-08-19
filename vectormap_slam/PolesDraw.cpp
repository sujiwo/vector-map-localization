/*
 * PolesDraw.cpp
 *
 *  Created on: Jul 25, 2015
 *      Author: sujiwo
 */

#include <PolesDraw.h>
#include "vector_map.h"
#include "RenderWidget.h"


PolesDraw::PolesDraw (VectorMap *vmap, RenderWidget *w) :
	VectorMapObject (w)
{
	for(std::map<int, Pole>::iterator it=vmap->poles.begin(); it!=vmap->poles.end(); it++) {
		Pole cpole = it->second;
		Point3 pbottom = vmap->getPoint(vmap->vectors[cpole.vid].pid);
		Point3 ptop = pbottom + Vector3 (0, 0, cpole.length);

		polePoints.push_back(pbottom);
		polePoints.push_back(ptop);
	}

	setColor(150, 150, 150);
}


void PolesDraw::initialize()
{
	initBuffer ();
	vbuffer->bind();
	vbuffer->allocate(polePoints.data(), sizeof(Point3)*polePoints.size());
	vbuffer->release();
}


void PolesDraw::draw()
{
	vbuffer->bind();
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	glLineWidth (3.0);
	glwidget->getShader()->setUniformValue("objColor", color2qv(color));
	glDrawArrays (GL_LINES, 0, polePoints.size());
	//glFlush ();

	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release();
}
