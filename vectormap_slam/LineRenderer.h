/*
 * LineRenderer.h
 *
 *  Created on: Sep 1, 2015
 *      Author: sujiwo
 */

#ifndef _LINERENDERER_H_
#define _LINERENDERER_H_


#include <vector>
#include "vector_map.h"
#include "Camera.h"


using std::vector;


class LineRenderer
{
public:
	LineRenderer (VectorMap *, Camera *);

	void render (Point3 &position, Quaternion &orientation, vector<float> &lines);

protected:
	VectorMap *map;
};

#endif /* _LINERENDERER_H_ */
