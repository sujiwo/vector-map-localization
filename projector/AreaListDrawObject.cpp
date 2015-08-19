/*
 * AreaListDrawObject.cpp
 *
 *  Created on: May 12, 2015
 *      Author: sujiwo
 */

#include "AreaListDrawObject.h"
#include "SceneManager.h"
#include "debug.h"


AreaListDrawObject::AreaListDrawObject (VectorMap &v) :
	vmap (v)
{
	glGenBuffers (1, &vbo);
	tessel = gluNewTess ();

	// register callback functions
	gluTessCallback (tessel, GLU_TESS_BEGIN_DATA, (void (CALLBACK *)())AreaListDrawObject::__tessBeginCB);
	gluTessCallback (tessel, GLU_TESS_END_DATA, (void (CALLBACK *)())AreaListDrawObject::__tessEndCB);
	gluTessCallback (tessel, GLU_TESS_VERTEX_DATA, (void (CALLBACK *)())AreaListDrawObject::__tessVertexCB);
}


AreaListDrawObject::~AreaListDrawObject ()
{
	glDeleteBuffers (1, &vbo);
	gluDeleteTess (tessel);
}


void AreaListDrawObject::initialize ()
{
	for(std::map<int, Area>::iterator it=vmap.areas.begin(); it!=vmap.areas.end(); it++) {
		Area polygon = it->second;

		gluTessBeginPolygon (tessel, this);
		gluTessBeginContour (tessel);

		Point3 shpt = vmap.getPoint(vmap.lines[polygon.slid].bpid);
		gluTessVertex (tessel, (GLdouble*)&shpt, this);

		for (int lid=polygon.slid; lid<polygon.elid; lid++) {
			Line curline = vmap.lines[lid];
			Point3 ptx = vmap.getPoint(curline.bpid);
			gluTessVertex (tessel, (GLdouble*)&ptx, this);
		}

		gluTessEndContour (tessel);
		gluTessEndPolygon (tessel);
	}

	numOfPoints = areaPoints.size();
	debug ("There are %d points", numOfPoints);

	// initialize Vertex Buffer
	glBindBuffer (GL_ARRAY_BUFFER, vbo);

	glBufferData (GL_ARRAY_BUFFER,
		sizeof(Point3)*areaPoints.size(),
		areaPoints.data(),
		GL_STATIC_DRAW);

	glBindBuffer (GL_ARRAY_BUFFER, 0);

	setColor (0xff, 0xcb, 0xdb);
}


void AreaListDrawObject::draw ()
{
	scene->getShader()->use();
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	doSetColor ();
	glDrawArrays (GL_TRIANGLES, 0, numOfPoints);

	glDisableClientState (GL_VERTEX_ARRAY);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}


void CALLBACK AreaListDrawObject::__tessBeginCB (GLenum type, void *a)
{
	AreaListDrawObject *area = (AreaListDrawObject*)a;
}


void CALLBACK AreaListDrawObject::__tessEndCB (void *a)
{
	AreaListDrawObject *area = (AreaListDrawObject*)a;
}


void CALLBACK AreaListDrawObject::__tessVertexCB (void *vtxdata, void *a)
{
	AreaListDrawObject *area = (AreaListDrawObject*)a;
	Point3 *vtx = (Point3*)vtxdata;
	area->areaPoints.push_back(*vtx);
}

