/*
 * PointSolver.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: sujiwo
 */

#include "PointSolver.h"
#include "RenderWidget.h"
#include "Camera.h"


const float farPlane = 50.0;


PointSolver::PointSolver (VectorMap *src, RenderWidget *gl, int w, int h) :
	map (src),
	glcanvas (gl),
	width (w), height (h)
{
	// re-create camera parameters
	// There may be differences with calculated camera matrix in Camera.cpp
	Camera::CameraIntrinsic &cameraParams = glcanvas->getCamera()->getCameraParam();
	camera = cv::Mat::zeros(3, 3, CV_32F);

//	camera.at<float>(0, 0) = cameraParams.fx;
//	camera.at<float>(0, 2) = cameraParams.cx;
//	camera.at<float>(1, 1) = cameraParams.fy;
//	camera.at<float>(1, 2) = cameraParams.cy;
//	camera.at<float>(2, 2) = 1;
}


void PointSolver::solve (cv::Mat *processedInputImage, Point3 &startPosition, Quaternion &startOrientation)
{
	assert (processedInputImage->type()==CV_8UC1);
	image = processedInputImage;
	position0 = startPosition;
	orientation0 = startOrientation;

	prepareImage ();
	projectLines ();
}


void PointSolver::prepareImage ()
{
	imagePoints.clear();

	for (int i=0; i<image->rows; i++) {
		uint8_t *p = image->ptr<uint8_t> (i);
		for (int j=0; j<image->cols; j++) {
			if (p[j] !=0 ) {
				ImagePoint pt;
				pt.px = j;
				pt.py = i;
				pt.nearestLine = -1;
				pt.lineDistance = -1;
				imagePoints.push_back (pt);
			}
		}
	}
}


void debugProjection (vector<Point2> &projResult)
{
	cv::Mat proj = cv::Mat::zeros (480, 640, CV_8UC1);
	for (int i=0; i<projResult.size(); i++) {
		Point2 point = projResult[i];
		if (point.x() >= 0 and point.x() <= 640 and point.y()>=0 and point.y()<=480) {
			int u = point.x(), v = point.y();
			proj.at<uint8_t> (v,u) = 255;
		}
	}

	cv::imwrite ("/tmp/debugprojection.png", proj);
}


void projectPoints_tf (Point3 &p0, Quaternion &o0, Camera::CameraIntrinsic &camera, VectorMap *map, vector<Point2> &result, int w, int h)
{
	tf::Vector3 ptf0 (p0.x(), p0.y(), p0.z());
	tf::Quaternion otf0 (o0.x(), o0.y(), o0.z(), o0.w());
	tf::Transform tfs (otf0, ptf0);

	double zoom_x = (double)w / (double)camera.wi,
		zoom_y = (double)h / (double)camera.wh;

	for (int i=1; i<=map->points.size(); i++) {
		Point3 p3 = map->getPoint (i);
		tf::Vector3 ptf (p3.x(), p3.y(), p3.z());
		tf::Vector3 ptfdst = tfs (ptf);

		Point2 pointScr (
			ptfdst.x()*camera.fx*zoom_x / ptfdst.z() + camera.cx*zoom_x,
			ptfdst.y()*camera.fy*zoom_y / ptfdst.z() + camera.cy*zoom_y);

		result.push_back (pointScr);
	}
}


void computeProjectionJacobian (
	Point3 &pos,			// Camera center coordinate
	Quaternion &ori,		// Camera orientation
	Point3 &point,			// Original point position
	Point3 &pcam,			// Point in camera coordinate
	Point2 &pim,			// Point in image
	float fx,
	float fy,
	float cx,
	float cy,	// From Intrinsic Matrix
	pscalar jacobian[7][2]	// jacobian result
	)
{

}


void PointSolver::projectLines ()
{
	visibleLines.clear();
	vector<Point2> result_tf;
	Camera::CameraIntrinsic camera = glcanvas->getCamera()->getCameraParam();

	tf::Vector3 ptf0 (position0.x(), position0.y(), position0.z());
	tf::Quaternion otf0 (orientation0.x(), orientation0.y(), orientation0.z(), orientation0.w());
	tf::Transform cameraEx (otf0, ptf0);

	double zoom_x = (double)width / (double)camera.wi,
		zoom_y = (double)height / (double)camera.wh;

	camera.fx *= zoom_x;
	camera.fy *= zoom_y;
	camera.cx *= zoom_x;
	camera.cy *= zoom_y;

	for(std::map<int, Line>::iterator it=map->lines.begin(); it!=map->lines.end(); it++) {
		Line l = it->second;

		// Projection from 3D to camera image plane
		Point3 A = map->getPoint (l.bpid),
			B = map->getPoint (l.fpid);
		tf::Vector3 PA (A.x(), A.y(), A.z()),
			PB (B.x(), B.y(), B.z());
		tf::Vector3 PAcam = cameraEx (PA),
			PBcam = cameraEx (PB);

		Point2 PAim (PAcam.x()*camera.fx/PAcam.z() + camera.cx, PAcam.y()*camera.fy/PAcam.z());
		Point2 PBim (PBcam.x()*camera.fx/PBcam.z() + camera.cx, PBcam.y()*camera.fy/PBcam.z());

		// filter these points
		if (PAcam.z()<0 and PBcam.z()<0)
			continue;
		if (PAcam.z()>farPlane and PBcam.z()>farPlane)
			continue;
		if ((PAim.x()<0 or PAim.x()>width or PAim.y()<0 or PAim.y()>width) and
			(PBim.x()<0 or PBim.x()>width or PBim.y()<0 or PBim.y()>width))
			continue;

		ProjectedPoint P1, P2;
		P1.coord = PAim;
		P1.mapPid = l.bpid;
		P2.coord = PBim;
		P2.mapPid = l.fpid;
		LineSegment2D lix;
		lix.A = P1; lix.B = P2;
		lix.mapLid = l.lid;

		Point3 _PAcam_ (PAcam.x(), PAcam.y(), PAcam.z());
		Point3 _PBcam_ (PBcam.x(), PBcam.y(), PBcam.z());
		computeProjectionJacobian (position0, orientation0, A, _PAcam_, PAim,
				camera.fx, camera.fy, camera.cx, camera.cy,
				P1.jacobian);
		computeProjectionJacobian (position0, orientation0, B, _PBcam_, PBim,
				camera.fx, camera.fy, camera.cx, camera.cy,
				P2.jacobian);

		visibleLines.push_back (lix);
	}

//	projectPoints_tf (position0, orientation0, cameraParams, map, result_tf, width, height);
//	debugProjection (result_tf);

	return;
}
