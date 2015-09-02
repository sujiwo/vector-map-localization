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
	int nearestLine;
	pscalar lineDistance;
};

struct ModelLine {
	Point3 p1, p2;
	ModelLine (Point3 A, Point3 B) {p1=A, p2=B;}
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

	// for debugging purposes
	void debugProjection (const char *imageFilename);

	static void projectModel (cv::Mat &outputImage, vector<ModelLine> &m, Camera *camera, int w, int h);

protected:
	int width, height;
	Camera *camera;
	Point3 position0;
	Quaternion orientation0;
	cv::Mat image;
	vector<ModelLine> model;
	vector<LineSegment2D> visibleLines;
	vector<ImagePoint> ipoints;

	void projectLines ();
	void prepareImage ();
	void prepareMatrices ();
	void solveForCorrection ();

	// solution matrices
	Eigen::MatrixXd Jac;
	Eigen::VectorXd Pcorrect, pointErrs;

};

#endif /* _POINTSOLVER2_H_ */
