/*
 * Tesselator.cpp
 *
 *  Created on: Aug 9, 2015
 *      Author: sujiwo
 */

#include "Tesselator.h"

#define GLDEBUG
#include "debug.h"



Tesselator::Tesselator()
{
	tessel = gluNewTess ();

	gluTessCallback (tessel, GLU_TESS_BEGIN_DATA, (void (*)())&Tesselator::tesselBeginCb);
	gluTessCallback (tessel, GLU_TESS_END_DATA, (void (*)())&Tesselator::tesselEndCb);
	gluTessCallback (tessel, GLU_TESS_VERTEX_DATA, (void (*)())&Tesselator::tesselVertexCb);
	gluTessCallback (tessel, GLU_TESS_ERROR, (void (*)())&Tesselator::tesselErrorCb);
	gluTessCallback (tessel, GLU_TESS_COMBINE_DATA, (void (*)())&Tesselator::tesselCombineCb);
}


Tesselator::~Tesselator()
{
	gluDeleteTess (tessel);
}


void CALLBACK Tesselator::tesselBeginCb (GLenum type, void *data)
{
	Tesselator *current = (Tesselator*)data;
	current->workp.drawType = type;
}


void CALLBACK Tesselator::tesselEndCb (void *data)
{
	Tesselator *current = (Tesselator*)data;
}


void CALLBACK Tesselator::tesselVertexCb (void *vtxdata, void *data)
{
	Tesselator *current = (Tesselator*)data;
	const GLdouble *ptr = (const GLdouble*)vtxdata;
	Point3 vtx (ptr[0], ptr[1], ptr[2]);
	current->workp.pointlist.push_back(vtx);
}


void CALLBACK Tesselator::tesselErrorCb (GLenum errorcode)
{
	//Tesselator *current = (Tesselator*)data;
	const unsigned char *err = gluErrorString (errorcode);
	debug ("GL Error (%d): %s", errorcode, err);
	throw err;
}


void CALLBACK Tesselator::tesselCombineCb (
	GLdouble cbvtx[3],
	void *vertex_data[4],
	GLfloat weight[4],
	GLdouble **outData,
	void *user_data)
{
//	debug ("Combined");
	Point3 newvtx (cbvtx[0], cbvtx[1], cbvtx[2]);
	Tesselator *current = (Tesselator*)user_data;
	current->workp.pointlist.push_back(newvtx);

	GLdouble *vtx = new GLdouble[3];
	vtx[0] = cbvtx[0];
	vtx[1] = cbvtx[1];
	vtx[2] = cbvtx[2];
	*outData = vtx;
}


Polygon Tesselator::tesselize (Area &area, VectorMap &map)
{
	vector<Point3> areaPoints = areaToPoints (area, map);
	return tesselize (areaPoints);
}


Polygon Tesselator::tesselize (vector<Point3> &pointlst)
{
	workp.clear ();

	gluTessBeginPolygon (tessel, this);
		gluTessBeginContour (tessel);
			for (int i=0; i<pointlst.size(); i++) {
				Point3 &ptx = pointlst[i];
				gluTessVertex (tessel, (GLdouble*)&ptx, this);
			}
		gluTessEndContour (tessel);
	gluTessEndPolygon (tessel);

	return workp;
}
