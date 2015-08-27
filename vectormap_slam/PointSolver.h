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


class PointSolver
{
public:
	PointSolver (VectorMap *src, RenderWidget *gl);

	// XXX: requires output
	void solve (cv::Mat *processedInputImage, Point3 &startPosition, Quaternion &startOrientation);


	struct ImagePoint {
		float px, py;
		int nearestLine;
		float lineDistance;
	};


private:
	VectorMap *map;
	RenderWidget *glcanvas;

	cv::Mat *image;
	Point3 position0;
	Quaternion orientation0;
	// Camera intrinsic parameters, required by OpenCV
	cv::Mat camera;

	// this vector is a list of all white points in source image
	vector<ImagePoint> imagePoints;
	// list of all lines currently visible when projected in image
	vector<Line> visibleLines;

	void prepareImage ();
	void projectLines ();
};

#endif /* POINTSOLVER_H_ */
