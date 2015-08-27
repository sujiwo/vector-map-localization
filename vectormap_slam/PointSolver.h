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




class PointSolver
{
public:
	PointSolver (VectorMap *src);

	// XXX: requires output
	void solve (cv::Mat *processedInputImage, Point3 &startPosition, Quaternion &startOrientation);


	struct ImagePoint {
		float px, py;
		int nearestLine;
	};


private:
	VectorMap *map;
	cv::Mat *image;
	Point3 position0;
	Quaternion orientation0;

	//void prepareImage
};

#endif /* POINTSOLVER_H_ */
