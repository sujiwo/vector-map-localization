/*
 * Tesselator.h
 *
 *  Created on: Aug 9, 2015
 *      Author: sujiwo
 */

#ifndef VECTORMAP_SLAM_TESSELATOR_H_
#define VECTORMAP_SLAM_TESSELATOR_H_


#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include "Math.h"
#include "vector_map.h"


using std::vector;


#ifndef CALLBACK
#define CALLBACK
#endif


struct Polygon {
	GLenum drawType;
	vector<Point3> pointlist;

	void clear ()
	{
		drawType = 0; pointlist.clear();
	}
};



class Tesselator {
public:
	Tesselator();
	virtual ~Tesselator();

	static void CALLBACK tesselBeginCb (GLenum type, void *data);
	static void CALLBACK tesselEndCb (void *data);
	static void CALLBACK tesselVertexCb (void *vtxdata, void *data);
	static void CALLBACK tesselErrorCb (GLenum errorcode);
	static void CALLBACK tesselCombineCb (GLdouble coords[3],
		void *vertex_data[4],
		GLfloat weight[4],
		GLdouble **outData,
		void *user_data);

	Polygon tesselize (Area &area, VectorMap &map);
	Polygon tesselize (vector<Point3> &pointlst);

private:
	GLUtesselator *tessel;
	Polygon workp;
};

#endif /* VECTORMAP_SLAM_TESSELATOR_H_ */
