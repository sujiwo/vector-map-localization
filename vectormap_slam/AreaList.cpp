/*
 * AreaList.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: sujiwo
 */

#include "AreaList.h"
#include "RenderWidget.h"
#include "vector_map.h"





AreaList::AreaList(VectorMap *v, RenderWidget *w) :
	VectorMapObject (w),
	map (v)
{
	this->loadArea ();
	setColor (128, 80, 0);
}


//void AreaList::loadArea (int areaId, vector<Point3> &pointsInArea)
//{
//	Area a = map->areas[areaId];
//	pointsInArea = areaToPoints (a, *map);
//}


void AreaList::loadArea ()
{
	int pid = 0;
	for(std::map<int, Area>::iterator it=map->areas.begin(); it!=map->areas.end(); it++) {
		Area &a = it->second;
		Polygon pl = tesselator.tesselize (a, *map);
		pointBuffer.insert (pointBuffer.end(), pl.pointlist.begin(), pl.pointlist.end());
		firstPointIndices.push_back(pid);
		polygonPointCounters.push_back(pl.pointlist.size());
		polygonTypes.push_back(pl.drawType);
		pid += pl.pointlist.size();
	}
}


void AreaList::initialize()
{
	initBuffer ();
	vbuffer->bind();
	int len = pointBuffer.size();
	vbuffer->allocate(pointBuffer.data(), sizeof(Point3)*len);
	vbuffer->release();
}


void AreaList::draw()
{
	vbuffer->bind();
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	glwidget->getShader()->setUniformValue("objColor", color2qv(color));

	for (int p=0; p<polygonTypes.size(); p++) {
		glDrawArrays (polygonTypes[p], firstPointIndices[p], polygonPointCounters[p]);
	}

	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release();
}
