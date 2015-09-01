/*
 * PointSolver2.h
 *
 *  Created on: Sep 1, 2015
 *      Author: sujiwo
 */

#ifndef _POINTSOLVER2_H_
#define _POINTSOLVER2_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include "Math.h"


class Camera;
typedef float pscalar;

using std::vector;


struct ImagePoint {
	Point2 coord;
	int lineId;
	pscalar lineDistance;
};

struct ModelLine {
	Point3 p1, p2;
};

struct ProjectedPoint {
	Point2 coord;
	int modelLid;
	pscalar jacobian[7][2];
};


struct LineSegment2D {
	ProjectedPoint A, B;
	int modelLid;

	pscalar error (Point2 &p);
	pscalar errorJacobian (Point2 &p, pscalar jacobianMat[7]);

	pscalar lengthSquared ()
	{ return (B.coord - A.coord).squaredNorm(); }

	pscalar length ()
	{ return (B.coord - A.coord).norm(); }

	pscalar distanceSquared (Point2 &p);
};



class PointSolver2
{
public:
	PointSolver2 (vector<ModelLine> &m, Camera *c, int w, int h);

	void solve (cv::Mat &inputImage, Point3 &startPos, Quaternion &startOrientation);

protected:
	int width, height;
	Camera *camera;
	Point3 pos;
	Quaternion ori;
	cv::Mat image;
	vector<ModelLine> model;
	vector<LineSegment2D> visibleLines;

	void projectLines ();
};

#endif /* _POINTSOLVER2_H_ */
