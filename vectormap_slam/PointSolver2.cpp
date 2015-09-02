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

	projectLines ();
	prepareImage ();
	prepareMatrices ();
}


bool projectPoint (Camera *camera, Point3 &src, Point2 &res, int width, int height)
{
	int u, v;
	if (camera->project (src, u, v, width, height)==true) {
		res = Point2 (u, height-v);
		return true;
	}
	else return false;
}


void projectPoint (Point3 &src, Point3 eyePos, Quaternion &orientation, Point2 &res, float fx, float fy, float cx, float cy, int width=0, int height=0)
{

}


void PointSolver2::projectLines()
{
	visibleLines.clear();

	for (int lid=0; lid<model.size(); lid++) {
		ModelLine &line = model[lid];

		ProjectedPoint P1, P2;
		if (projectPoint (camera, line.p1, P1.coord, width, height)==false or
			projectPoint (camera, line.p2, P2.coord, width, height)==false) {
			continue;
		}

		LineSegment2D vLine;
		vLine.A = P1, vLine.B = P2;
		vLine.modelLid = lid;

		visibleLines.push_back (vLine);
	}
}


void PointSolver2::prepareImage ()
{
	ipoints.clear ();

	for (int i=0; i<image.rows; i++) {
		uint8_t *p = image.ptr<uint8_t> (i);
		for (int j=0; j<image.cols; j++) {
			if (p[j] !=0 ) {
				ImagePoint pt;
				pt.coord.x() = j;
				pt.coord.y() = i;
				pt.nearestLine = -1;
				pt.lineDistance = -1;
				// imagePoints.push_back (pt);

				pt.lineDistance = 1e32;

				// find nearest line
				for (int il=0; il<visibleLines.size(); il++) {
					LineSegment2D &line = visibleLines[il];
					pscalar dist = line.distanceSquared(pt.coord);
					if (dist < pt.lineDistance) {
						pt.nearestLine = il;
						pt.lineDistance = dist;
					}
				}

				if (pt.lineDistance < 4000.0) {
					ipoints.push_back (pt);
				}
			}
		}
	}
}


void PointSolver2::projectModel(cv::Mat &output, vector<ModelLine> &model, Camera *camera, int width, int height)
{
	output = cv::Mat::zeros(height, width, CV_8UC1);

	for (int lid=0; lid<model.size(); lid++) {
		ModelLine &line = model[lid];

		ProjectedPoint P1, P2;
		if (projectPoint (camera, line.p1, P1.coord, width, height)==false or
			projectPoint (camera, line.p2, P2.coord, width, height)==false) {
			continue;
		}

		cv::Point p1v (P1.coord.x(), P1.coord.y());
		cv::Point p2v (P2.coord.x(), P2.coord.y());
		cv::circle (output, p1v, 3, 255);
		cv::circle (output, p2v, 3, 255);
		cv::line (output, p1v, p2v, 255);
	}
}


void PointSolver2::prepareMatrices ()
{

}
