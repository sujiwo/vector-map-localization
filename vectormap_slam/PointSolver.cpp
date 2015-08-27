/*
 * PointSolver.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: sujiwo
 */

#include <PointSolver.h>

PointSolver::PointSolver(VectorMap *src) :
	map (src)
{}


void PointSolver::solve (cv::Mat *processedInputImage, Point3 &startPosition, Quaternion &startOrientation)
{
	image = processedInputImage;
	position0 = startPosition;
	orientation0 = startOrientation;
}
