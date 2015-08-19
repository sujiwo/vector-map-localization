/*
 * VectorMapDrawObject.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */

#include "VectorMapDrawObject.h"

VectorMapDrawObject::VectorMapDrawObject(VectorMap *m) :
	map (m)
{
	glGenBuffers (1, &vbo);
}


VectorMapDrawObject::~VectorMapDrawObject()
{
	glDeleteBuffers (1, &vbo);
}


void VectorMapDrawObject::initialize()
{
	glBindBuffer (GL_ARRAY_BUFFER, vbo);

	int ptlen = map->points.size();
	for (int i=0; i<ptlen; i++) {
		PointXYZ p = map->getPoint(i);
		points.push_back(p);
	}
	// watch out for mismatch size
	glBufferData (GL_ARRAY_BUFFER, sizeof(PointXYZ)*ptlen, points.data(), GL_STATIC_DRAW);

	glBindBuffer (GL_ARRAY_BUFFER, 0);
}


void VectorMapDrawObject::draw()
{
	shader->use();
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 4*sizeof(float), 0);

	setColor(Vector3(1.0, 1.0, 0));
	glDrawArrays (GL_POINTS, 0, points.size());

	glDisableClientState (GL_VERTEX_ARRAY);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}
