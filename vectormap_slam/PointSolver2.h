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
#include <limits>
#include <cstring>
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
	Point3 inCam;
	int modelLid;
	pscalar jacobian[7][2];
};


struct LineSegment2D {
	ProjectedPoint A, B;
	int modelLid;
	pscalar sin, cos, d;

	LineSegment2D (const ProjectedPoint &pA, const ProjectedPoint &pB, int &lid) :
		A(pA), B(pB), modelLid(lid)
	{
		auto r = length();
		sin = (B.coord.x() - A.coord.x()) / r;
		cos = (B.coord.y() - A.coord.y()) / r;
		d = A.coord.x()*sin - A.coord.y()*cos;
		memset (A.jacobian, 0, sizeof(A.jacobian));
		memset (B.jacobian, 0, sizeof(B.jacobian));
	}

	pscalar error (Point2 &p);
	void errorJacobian (Point2 &p, pscalar jacobianMat[7]);

	pscalar lengthSquared ()
	{ return (B.coord - A.coord).squaredNorm(); }

	pscalar length ()
	{ return (B.coord - A.coord).norm(); }

	pscalar distanceSquared (Point2 &p);

	pscalar distance (const Point2 &p)
	{
		auto da = p.x()*sin - p.y()*cos;
		return da - d;
	}

	Point2 center()
	{ return (A.coord + B.coord) / 2; }

	// Returns a point in this line which is nearest to a specified point
	Point2 nearestTo (const Point2 &t);

};




class PointSolver2
{
public:


	struct Projector {
		Projector (pscalar fx, pscalar fy, pscalar cx, pscalar cy, int width, int height);
		Projector (pscalar angleDegree, int width, int height);
		Point2 operator * (Point4 &pointInCam);

		pscalar fx () { return matrix(0, 0); }
		pscalar fy () { return matrix(1, 1); }
		pscalar cx () { return matrix(0, 2); }
		pscalar cy () { return matrix(1, 2); }

//		Eigen::Matrix<pscalar, 3, 4> matrix;
		Matrix4 matrix;
		pscalar width, height;

		inline Point2 ScreenToNormalized (const pscalar u, const pscalar v)
		{ return Point2 (2*u/width - 1 , 1 - 2*v/height); }

		inline Point2 ScreenToNormalized (const Point2& scr)
		{ return ScreenToNormalized (scr.x(), scr.y()); }

		inline Point2 NormalizedToScreen (const pscalar x, const pscalar y)
		{ return Point2 (width*(x+1)/2, height*(1-y)/2); }

		inline Point2 NormalizedToScreen (const Point2& nrm)
		{ return NormalizedToScreen (nrm.x(), nrm.y()); }
	};


	PointSolver2 (vector<ModelLine> &m, PointSolver2::Projector &proj);

	void solve (cv::Mat &inputImage, Point3 &startPos, Quaternion &startOrientation);

	// for debugging purposes
	void debugProjection (const char *imageFilename);
	void debugPointPairing (const char *imgname);
	void debugDraw (const char *imgname, Point3 *pos=NULL, Quaternion *orin=NULL);

	static void projectModel (cv::Mat &outputImage, vector<ModelLine> &m, PointSolver2::Projector &projector, Matrix4 &viewMatrix);
	static void projectModel (cv::Mat &outputImage, vector<ModelLine> &m, PointSolver2::Projector &projector, Point3 &cameraPosition, Quaternion &cameraOrientation);
	static void projectModel (cv::Mat &outputImage, vector<ModelLine> &m, PointSolver2::Projector &projector, Point3 &cameraPosition, Point3 &centerOfView, Vector3 &up);


protected:
	int width, height;
	Projector projectionMatrix;
	Point3 position0;
	Quaternion orientation0;
	cv::Mat image;
	vector<ModelLine> model;
	vector<LineSegment2D> visibleLines;
	vector<ImagePoint> ipoints;
	Matrix4 currentViewMatrix;

	void prepareImage ();

	void projectLines ();
	void findNearestLinePerPoint ();

	void prepareMatrices ();
	void solveForCorrection ();

	double calcCurrentError ();

	// solution matrices
	Eigen::MatrixXd Jac;
	Eigen::VectorXd Pcorrect, pointErrs;

};


Matrix4 createViewMatrix (Point3 &cameraPosition, Quaternion &cameraOrientation);
Matrix4 createViewMatrix (Point3 &cameraPosition, Point3 &centerOfView, Vector3 &_up);
void poseFromViewMatrix (Matrix4 &viewMatrix, Point3 &position, Quaternion &orientation);


#endif /* _POINTSOLVER2_H_ */
