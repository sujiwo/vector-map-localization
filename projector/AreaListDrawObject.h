/*
 * AreaListDrawObject.h
 *
 *  Created on: May 12, 2015
 *      Author: sujiwo
 */

#ifndef SOURCE_DIRECTORY__PROJECTOR_AREALISTDRAWOBJECT_H_
#define SOURCE_DIRECTORY__PROJECTOR_AREALISTDRAWOBJECT_H_

#include "DrawObject.h"
#include "vector_map.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>


using std::vector;


#ifndef CALLBACK
#define CALLBACK
#endif


class AreaListDrawObject: public DrawObject
{
public:
	AreaListDrawObject (VectorMap &v);
	virtual ~AreaListDrawObject ();

	void initialize ();
	void draw ();

	typedef shared_ptr<AreaListDrawObject> Ptr;

	// tesselation stuffs
	static void CALLBACK __tessBeginCB (GLenum type, void *data);
	static void CALLBACK __tessEndCB (void *data);
	static void CALLBACK __tessVertexCB (void *vtxdata, void *data);

protected:
	VectorMap &vmap;
	GLuint vbo;
	int numOfPoints;
	vector<Point3> areaPoints;
	GLUtesselator *tessel;
};

#endif /* SOURCE_DIRECTORY__PROJECTOR_AREALISTDRAWOBJECT_H_ */
