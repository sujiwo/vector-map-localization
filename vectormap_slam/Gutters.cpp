/*
 * Gutters.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: sujiwo
 */

#include "Gutters.h"
#include "vector_map.h"
#include "debug.h"
#include <iostream>
#include <fstream>
#include <string>
#include "RenderWidget.h"


using std::ifstream;
using std::string;


Gutters::Gutters(VectorMap *v, RenderWidget *w) :
	VectorMapObject (w),
	map (v)
{
	setColor (188, 157, 157);

	string fname = map->dirname + "/gutter.csv";
	ifstream gutterfd (fname);

	int lptr = 0;
	while (gutterfd.good()) {
		string line;
		int id, aid, ty, linkId;
		getline (gutterfd, line);
		sscanf (line.c_str(), "%d,%d,%d,%d", &id, &aid, &ty, &linkId);
		if (lptr > 0) {
			areaIds.push_back(aid);
		}
		lptr++;
	}
}


void Gutters::initialize ()
{
	initBuffer ();
	vbuffer->bind();

	int pid=0;
	for (int i=0; i<areaIds.size(); i++) {
		Area a = map->areas[areaIds[i]];
		vector<Point3> pointsInArea = areaToPoints (a, *map);
		pointBuffer.insert (pointBuffer.end(), pointsInArea.begin(), pointsInArea.end());

		vector<uint32_t> pidxs;
		for (uint32_t i=pid; i<pid+pointsInArea.size(); i++)
			pidxs.push_back(i);
		pointIndex.push_back(pidxs);
		pid += pointsInArea.size();
	}

	vbuffer->release();
}


void Gutters::draw ()
{
	vbuffer->bind();
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	glwidget->getShader()->setUniformValue("objColor", color2qv(color));
	for (int aid=0; aid<pointIndex.size(); aid++) {
		vector<uint32_t> cAreaPtIdx = pointIndex[aid];
		glDrawElements (GL_POLYGON, cAreaPtIdx.size(), GL_UNSIGNED_INT, cAreaPtIdx.data());
	}

	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release();
}
