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
	void errorJacobian (Point2 &p, pscalar jacobianMat[7]);

	pscalar lengthSquared ()
	{ return (B.coord - A.coord).squaredNorm(); }

	pscalar length ()
	{ return (B.coord - A.coord).norm(); }

	pscalar distanceSquared (Point2 &p);

	Point2 center()
	{ return (A.coord + B.coord) / 2; }

	// Returns a point in this line which is nearest to a specified point
	Point2 nearestTo (const Point2 &t);

private:
	void errorJacobian1 (
		const float &px,
		const float &py,
		const float &p1x,
		const float &p1y,
		const float &p2x,
		const float &p2y,
		const float lsegmentsq,
		float &dep1x, float &dep1y, float &dep2x, float &dep2y);
	void errorJacobian2 (
		const float &px,
		const float &py,
		const float &p1x,
		const float &p1y,
		const float &p2x,
		const float &p2y,
		const float lsegmentsq,
		float &dep1x, float &dep1y, float &dep2x, float &dep2y);
	void errorJacobian3 (
		const float &px,
		const float &py,
		const float &p1x,
		const float &p1y,
		const float &p2x,
		const float &p2y,
		const float lsegmentsq,
		float &dep1x, float &dep1y, float &dep2x, float &dep2y);
};




class PointSolver2
{
public:


	struct Projector {
		Projector (pscalar fx, pscalar fy, pscalar cx, pscalar cy);
		Projector (pscalar angleDegree, int width, int height);
		Point2 operator * (Point4 &pointInCam);

		pscalar fx () { return matrix(0, 0); }
		pscalar fy () { return matrix(1, 1); }
		pscalar cx () { return matrix(0, 2); }
		pscalar cy () { return matrix(1, 2); }

		Eigen::Matrix<pscalar, 3, 4> matrix;
		pscalar width, height;
	};


	struct ProjectorNormalized : public Projector {

	};


	PointSolver2 (vector<ModelLine> &m, PointSolver2::Projector &proj);

	void solve (cv::Mat &inputImage, Point3 &startPos, Quaternion &startOrientation);

	// for debugging purposes
	void debugProjection (const char *imageFilename);
	void debugPointPairing (const char *imgname);

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

	void projectLines ();
	void prepareImage ();
	void prepareMatrices ();
	void solveForCorrection ();

	// solution matrices
	Eigen::MatrixXd Jac;
	Eigen::VectorXd Pcorrect, pointErrs;

};


Matrix4 createViewMatrix (Point3 &cameraPosition, Quaternion &cameraOrientation);
Matrix4 createViewMatrix (Point3 &cameraPosition, Point3 &centerOfView, Vector3 &_up);
void poseFromViewMatrix (Matrix4 &viewMatrix, Point3 &position, Quaternion &orientation);


#endif /* _POINTSOLVER2_H_ */
