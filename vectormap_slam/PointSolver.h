/*
 * PointSolver.h
 *
 *  Created on: Aug 27, 2015
 *      Author: sujiwo
 */

#ifndef POINTSOLVER_H_
#define POINTSOLVER_H_


#include "vector_map.h"
#include "Math.h"
#include <vector>
#include <opencv2/opencv.hpp>



using std::vector;
class RenderWidget;

typedef float pscalar;



class PointSolver
{
public:
	PointSolver (VectorMap *src, RenderWidget *gl, int targetWidth, int targetHeight);

	// XXX: requires output
	void solve (cv::Mat *processedInputImage, Point3 &startPosition, Quaternion &startOrientation);


	struct ImagePoint {
		float px, py;
		int nearestLine;
		float lineDistance;
	};


	struct ProjectedPoint {
		Point2 coord;
		int mapPid;
		pscalar jacobian[7][2];
	};


private:
	VectorMap *map;
	RenderWidget *glcanvas;

	cv::Mat *image;
	Point3 position0;
	Quaternion orientation0;
	// Camera intrinsic parameters, required by OpenCV
	cv::Mat camera;
	int width, height;

	// this vector is a list of all white points in source image
	vector<ImagePoint> imagePoints;
	// list of all lines currently visible when projected in image
	vector<Line> visibleLines;

	void prepareImage ();
	void projectLines ();

	void computeJacobian ();
};

#endif /* POINTSOLVER_H_ */
