/*
 * PointSolver2.cpp
 *
 *  Created on: Sep 1, 2015
 *      Author: sujiwo
 */

#include "PointSolver2.h"
#include "Camera.h"


PointSolver2::PointSolver2 (vector<ModelLine> &modelIn, Camera *c, int w, int h) :
	camera (c),
	width (w), height (h),
	model(modelIn)
{}


void PointSolver2::solve (cv::Mat &inputImage, Point3 &startPos, Quaternion &startOrientation)
{
	pos = startPos;
	ori = startOrientation;
	image = inputImage;
}
