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


struct ImagePoint {
	Point2 coord;
	int lineId;
	pscalar lineDistance;
};

struct ModelLine {
	Point3 p1, p2;
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
	vector<ModelLine> &model;
};

#endif /* _POINTSOLVER2_H_ */
