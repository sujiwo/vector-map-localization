/*
 * TrafficLights.cpp
 *
 *  Created on: Apr 13, 2015
 *      Author: sujiwo
 */

#include <TrafficLights.h>
#include "RenderWidget.h"


// convert double angle to 3D vector
Vector3 vectormapToVector3 (double Hang, double Vang)
{
	float x = sin (Hang),
		y = cos (Hang),
		z = cos (Vang);
	Vector3 v (x, y, z);
	v.normalize();
	return v;
}



TrafficLights::TrafficLights (VectorMap *v, RenderWidget *w) :
	VectorMapObject (w),
	vmap (v)
{
	int ipts = 0;

	// check if maximum limit of this loop is right
	for (int i=1; i<vmap->signals.size()-1; i++) {
		Signal signal = vmap->signals[i];
		int pid = vmap->vectors[signal.vid].pid;
		Point3 pxyz = vmap->getPoint(pid);
		signalCenterPoints.push_back (pxyz);

		TrafficLight scirc;
		scirc.id = signal.id;
		switch (signal.type) {
		case 1: scirc.color = SIGNAL_RED; break;
		case 2: scirc.color = SIGNAL_GREEN; break;
		case 3: scirc.color = SIGNAL_YELLOW; break;
		}
		scirc.points = signalPoint2Circle (vmap, signal);
		scirc.points.push_back (pxyz);
		for (int j=0; j<scirc.points.size(); j++) {
			scirc.idx.push_back(ipts);
			ipts++;
		}
		allPts.insert(allPts.end(), scirc.points.begin(), scirc.points.end());

		Vector v = vmap->vectors[signal.vid];
		scirc.direction = vectormapToVector3 (v.hang, v.vang);

		signalObjects.push_back(scirc);
	}
}


void TrafficLights::initialize ()
{
	initBuffer ();
	vbuffer->bind();
	vbuffer->allocate (allPts.data(), allPts.size()*sizeof(Point3));
	vbuffer->release();
}


void TrafficLights::draw()
{
	vbuffer->bind ();
	glEnableClientState (GL_VERTEX_ARRAY);
	glVertexPointer (3, GL_FLOAT, 0, 0);

	Vector3 vdir = glwidget->pose()->getDirectionVector();

	for (int i=0; i<signalObjects.size(); i++) {
		TrafficLight &signal = signalObjects[i];

//		double costheta = vdir.dot(signal.direction);
//		if (costheta > -0.5*M_PI)
//			continue;

		switch (signal.color) {
		case SIGNAL_RED:
			setColor(255, 0, 0); break;
		case SIGNAL_GREEN:
			setColor(0, 255, 0); break;
		case SIGNAL_YELLOW:
			setColor(255, 255, 0); break;
		}
		glwidget->getShader()->setUniformValue("objColor", color2qv(color));
		glDrawElements (GL_TRIANGLE_FAN, signal.idx.size(), GL_UNSIGNED_INT, signal.idx.data());
	}

	glDisableClientState (GL_VERTEX_ARRAY);
	vbuffer->release();
}


Vector3 TrafficLights::angle2UnitVector (const float Hang, const float Vang)
{
	Vector3 unv;
	unv.x() = sin (Hang * M_PI/180.0);
	unv.y() = cos (Hang * M_PI/180.0);
	unv.z() = 0.0;
	unv.normalize();
	return unv;
}


vector<Point3> TrafficLights::signalPoint2Circle (const Point3 &c, float Hang, float Vang)
{
	vector<Point3> circle;

	for (double angle=-M_PI; angle<M_PI; angle+=M_PI/8.0) {

		Point3 p;
		double u=0.3*cos(angle);
		double v=0.3*sin(angle);
		p.x() = c.x() + u*cos(Hang*M_PI/180.0);
		p.y() = c.y() - u*sin(Hang*M_PI/180.0);
		p.z() = c.z() + v;
		circle.push_back (p);

		u=0.3*cos(angle+M_PI/5);
		v=0.3*sin(angle+M_PI/5);
		p.x() = c.x() + u*cos(Hang*M_PI/180.0);
		p.y() = c.y() - u*sin(Hang*M_PI/180.0);
		p.z() = c.z() + v;
		circle.push_back (p);
	}

	return circle;
}


vector<Point3> TrafficLights::signalPoint2Circle (VectorMap *map, const Signal &sgn)
{
	Vector v = map->vectors[sgn.vid];
	int pid = v.pid;
	Point3 pc = map->getPoint(pid);
	return signalPoint2Circle (pc, v.hang, v.vang);
}


Point3 multiply3to4 (const Matrix4 &mat, const Point3 &point, float w)
{
	Point4 pw (point.x(), point.y(), point.z(), w);
	Point4 r4 = mat * pw;
	return Point3 (r4.x(), r4.y(), r4.z());
}


Point4 p3to4 (const Point3 &p3)
{
	return Point4 (p3.x(), p3.y(), p3.z(), 1);
}


Point3 p4to3 (const Point4 &p4)
{
	return Point3 (p4.x(), p4.y(), p4.z());
}


