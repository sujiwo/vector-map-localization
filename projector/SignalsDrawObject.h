/*
 * SignalsDrawObject.h
 *
 *  Created on: Apr 13, 2015
 *      Author: sujiwo
 */

#ifndef SIGNALSDRAWOBJECT_H_
#define SIGNALSDRAWOBJECT_H_

#include "DrawObject.h"
#include "vector_map.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <vector>
#include <stdint.h>


using std::vector;


const float lampRadius = 0.3;


#define SIGNAL_RED 1
#define SIGNAL_GREEN 2
#define SIGNAL_YELLOW 3


class Camera;


struct SignalCircle {
	Point3 center;
	vector<Point3> points;
	vector<unsigned int> idx;
	unsigned int color;
	unsigned int id;

	bool project (Camera &c, int &u, int &v, float &radInPixel);
	Point3 transform (Camera &c);
};



class SignalsDrawObject: public DrawObject
{
public:
	SignalsDrawObject (VectorMap &v, bool gl=true);
	virtual ~SignalsDrawObject();

	void initialize ();

	void draw ();

	typedef shared_ptr<SignalsDrawObject> Ptr;

	vector<SignalCircle> &getSignalList () { return signalObjects; }

private:
	VectorMap &vmap;
	vector<Point3> signalCenterPoints;

	vector<Point3> allPts;
	vector<SignalCircle> signalObjects;

	GLuint vbo;
	static Vector3 angle2UnitVector (const float Hang, const float Vang);
	static vector<Point3> signalPoint2Circle (const Point3 &centerPoint, float Hang, float Vang);
	static vector<Point3> signalPoint2Circle (VectorMap &map, const Signal &sgn);

};

#endif /* SIGNALSDRAWOBJECT_H_ */
