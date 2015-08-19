/*
 * VectorMapDrawObject.h
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */

#ifndef VECTORMAPDRAWOBJECT_H_
#define VECTORMAPDRAWOBJECT_H_

#include "DrawObject.h"
#include "vector_map.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <vector>


using std::vector;


class VectorMapDrawObject: public DrawObject
{
public:
	VectorMapDrawObject (VectorMap *m);
	virtual ~VectorMapDrawObject();

	void initialize ();
	void draw ();

private:
	VectorMap *map;
	GLuint vbo;
	vector<PointXYZ> points;
};

#endif /* VECTORMAPDRAWOBJECT_H_ */
