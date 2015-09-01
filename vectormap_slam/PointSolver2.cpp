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
	assert (inputImage.type() == CV_8UC1);

	pos = startPos;
	ori = startOrientation;
	image = inputImage;


}


void PointSolver2::projectLines()
{
	visibleLines.clear();

	for (int lid=0; lid<model.size(); lid++) {
		ModelLine &line = model[lid];


	}
}
