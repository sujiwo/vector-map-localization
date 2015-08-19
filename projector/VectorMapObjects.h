/*
 * VectorMapObjects.h
 *
 *  Created on: Jul 8, 2015
 *      Author: sujiwo
 */

#ifndef VECTORMAPOBJECTS_H_
#define VECTORMAPOBJECTS_H_

#include "Math.h"
#include <QtOpenGL/QGLBuffer>
#include <QtGui/qvector4d.h>
#include <string>


class VectorMapRenderWidget;
using std::string;


class VectorMapObject {
public:
	VectorMapObject(VectorMapRenderWidget *w);
	virtual ~VectorMapObject();

	virtual void initialize() = 0;
	virtual void draw() = 0;

	inline void setColor (const Vector3 &c)
	{ color = Vector4 (c[0], c[1], c[2], 1.0); }

	inline void setColor (
		const float &r,
		const float &g,
		const float &b)
	{ setColor (Vector3 (r, g, b)); }

	inline void setColor (int r, int g, int b)
	{ setColor (Vector3 ((1.0f/255)*r, (1.0f/255)*g, (1.0f/255)*b)); }


protected:
	Vector4 color;
	QGLBuffer *vbuffer;
	VectorMapRenderWidget *glwidget;
	void initBuffer ();
};



#include <vector>
using std::vector;


class PointList: public VectorMapObject {
public:
	PointList (VectorMapRenderWidget *w) :
		VectorMapObject(w), size(1.0) {}

	void initialize ();
	void draw ();

	inline void add (const Point3 &p)
	{ pointsrc.push_back(p); }

	inline void setPointSize (float f) {size=f;}

protected:
	vector<Point3> pointsrc;
	float size;
};


class TRectangle: public VectorMapObject {
public:
	TRectangle (const Point3 &p1, const Point3 &p2, const Point3 &p3, const Point3 &p4, VectorMapRenderWidget *w);
	void initialize ();
	void draw ();

private:
	Point3 points[4];
};



class VectorMap;

class MapPoints : public PointList
{
public:
	MapPoints (const string &path, VectorMapRenderWidget *w);
	~MapPoints ();
//	void initialize ();
//	void draw ();

private:
	VectorMap *map;
};


inline QVector4D color2qv (const Vector4 &v)
{
	return QVector4D (v.x(), v.y(), v.z(), v.w());
}


#endif /* VECTORMAPOBJECTS_H_ */
