/*
 * PointSolver.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: sujiwo
 */

#include "PointSolver.h"
#include "RenderWidget.h"
#include "Camera.h"
#include <iostream>
#include <fstream>
#include <string>


using std::string;
const float farPlane = 50.0;


PointSolver::PointSolver (VectorMap *src, RenderWidget *gl, int w, int h) :
	map (src),
	glcanvas (gl),
	width (w), height (h)
{
	// re-create camera parameters
	// There may be differences with calculated camera matrix in Camera.cpp
	Camera::CameraIntrinsic &cameraParams = glcanvas->getCamera()->getCameraParam();
}


void PointSolver::solve (cv::Mat *processedInputImage, Point3 &startPosition, Quaternion &startOrientation)
{
	assert (processedInputImage->type()==CV_8UC1);
	image = processedInputImage;
	position0 = startPosition;
	orientation0 = startOrientation;

	prepareImage ();
	projectLines ();
	debugProjection (visibleLines);
	pairPointsWithLines ();
	prepareMatrices ();
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


void PointSolver::debugProjection (vector<PointSolver::LineSegment2D> &projResult)
{
	std::fstream dumpline;
	dumpline.open ("/tmp/dumpl.txt", std::fstream::out);

	cv::Mat proj = cv::Mat::zeros (480, 640, CV_8UC1);
	for (int i=0; i<projResult.size(); i++) {
		Point2 p1 = projResult[i].A.coord,
				p2 = projResult[i].B.coord;

		int u1, v1, u2, v2;
		if (p1.x() >= 0 and p1.x() <= 640 and p1.y()>=0 and p1.y()<=480) {
			u1 = p1.x(), v1 = p1.y();
			proj.at<uint8_t> (v1,u1) = 255;
		}
		if (p2.x() >= 0 and p2.x() <= 640 and p2.y()>=0 and p2.y()<=480) {
			u2 = p2.x(), v2 = p2.y();
			proj.at<uint8_t> (v2,u2) = 255;
		}
		cv::line (proj, cv::Point2i(p1.x(),p1.y()), cv::Point2i(p2.x(),p2.y()), CV_RGB(255,255,255));
		// XXX: dump these line segments and plot in opencv
		dumpline << p1.x() << "," << p1.y() << "," << p2.x() << "," << p2.y() << "," << projResult[i].mapLid << std::endl;
	}

	cv::imwrite ("/tmp/debugprojection1.png", proj);

	dumpline << "# Orientation: " << orientation0.w() << "," << orientation0.x() << "," << orientation0.y() << "," << orientation0.z() << std::endl;
	dumpline << "# Position: " << position0 << std::endl;
	dumpline.close();
}


void computeProjectionJacobian (
	Point3 &t,			// Camera center coordinate
	Quaternion &q,		// Camera orientation
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
	pscalar Z = pcam.z();
	pscalar x = point.x(),
			y = point.y(),
			z = point.z();
	pscalar
		Tx = 2 * (q.w()*q.y() - q.x()*q.z()),
		Ty = -2 * (q.y()*q.z() + q.w()*q.x()),
		Tz = 2*q.y()*q.y() + 2*q.x()*q.x() - 1,
		Qx = -4*q.x()*(z-t.z()) + 2*q.w()*(y-t.y()) + 2*q.z()*(x-t.x()),
		Qy = -4*q.y()*(z-t.z()) + 2*q.z()*(y-t.y()) - 2*q.w()*(x-t.x()),
		Qz = 2*q.y()*(y - t.y()) + 2*q.x()*(x - t.x()),
		Qw = 2*q.x()*(y - t.y()) - 2*q.y()*(x - t.x());

	// dtxx
	jacobian[0][0] = ((fx*(2*q.z()*q.z() + 2*q.y()*q.y()-1) + cx*Tx) / Z) - Tx*pim.x()/Z;
	// dtxy
	jacobian[0][1] = ((cy*Tx + fy*(-2*q.w()*q.z()-2*q.x()*q.y())) / Z) - Tx*pim.y()/Z;
	// dtyx
	jacobian[1][0] = ((cx*Ty+fx*(2*q.w()*q.z()-2*q.x()*q.y())) / Z) - Ty*pim.x()/Z;
	// dtyy
	jacobian[1][1] = ((fy*(2*q.z()*q.z() + 2*q.x()*q.x() - 1) + cy*Ty) / Z) - Ty*pim.y()/Z;
	// dtzx
	jacobian[2][0] = ((fx*(-2*q.x()*q.z()-2*q.w()*q.y()) + cx*Tz) / Z) - ((Tz*pim.x())/Z);
	// dtzy
	jacobian[2][1] = ((fx*(-2*q.x()*q.z()-2*q.w()*q.y()) + cx*Tz) / Z) - ((Tz*pim.y())/Z);
	// dqxx
	jacobian[3][0] = ((fx*(2*q.z()*(z-t.z()) + 2*q.y()*(y-t.y())) + cx*Qx)/Z) - (Qx*pim.x()/Z);
	// dqxy
	jacobian[3][1] = ((cy*Qx + fy*(-2*q.w()*(z-t.z()) -4*q.x()*(y-t.y()) + 2*q.y()*(x-t.x())) )/Z) - (Qx*pim.y()/Z);
	// dqyx
	jacobian[4][0] = (cx*Qy + fx*(2*q.w()*(z-t.z()) +2*q.x()*(y-t.y()) -4*q.y()*(x-t.x())))/Z  -  Qy*pim.x()/Z;
	// dqyy
	jacobian[4][1] = (fy*(2*q.z()*(z-t.z()) + 2*q.x()*(x-t.x())) + cy*Qy)/Z  -  Qy*pim.y()/Z;
	// dqzx
	jacobian[5][0] = (fx*(2*q.x()*(z-t.z()) - 2*q.w()*(y-t.y()) - 4*q.z()*(x-t.x())) + cx*Qz)/Z - Qz*pim.x()/Z;
	// dqzy
	jacobian[5][1] = (fy*(2*q.y()*(z-t.z()) - 4*q.z()*(y-t.y()) + 2*q.w()*(x-t.x())) + cy*Qz)/Z - Qz*pim.y()/Z;
	// dqwx
	jacobian[6][0] = (fx*(2*q.y()*(z-t.z()) - 2*q.z()*(y-t.y())) + cx*Qw)/Z - Qw*pim.x()/Z;
	// dqwy
	jacobian[6][1] = (fy*(2*q.z()*(x-t.x()) - 2*q.x()*(z-t.z())) + cy*Qw)/Z - Qw*pim.y()/Z;
}


void projectAllPoints (Camera *camera, VectorMap *map, int w, int h)
{
	vector<Point2> result;

	for (int i=1; i<=map->points.size(); i++) {
		int u, v;
		Point3 P = map->getPoint (i);
		if (camera->project (P, u, v, w, h, false) == true) {
			Point2 pj (u, v);
			result.push_back(pj);
		}
	}

	cv::Mat proj = cv::Mat::zeros (h, w, CV_8UC1);
	for (int i=0; i<result.size(); i++) {
		Point2 P = result[i];
		cv::circle (proj, cv::Point2i(P.x(), P.y()), 2, 255);
	}

	cv::imwrite ("/tmp/debugprojection2.png", proj);
}


void projectAllPoints (Point3 &p0, Quaternion &o0, Camera::CameraIntrinsic &camera, VectorMap *map, int w, int h)
{
	vector<Point2> result;

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

	cv::Mat proj = cv::Mat::zeros (480, 640, CV_8UC1);
	for (int i=0; i<result.size(); i++) {
		Point2 p1 = result[i];
		if (p1.x() >= 0 and p1.x() <= 640 and p1.y()>=0 and p1.y()<=480) {
			int u = p1.x(), v = p1.y();
			//proj.at<uint8_t> (v,u) = 255;
			cv::circle (proj, cv::Point2i(u,v), 2, 255);
		}
	}

	cv::imwrite ("/tmp/debugprojection2.png", proj);
}


void PointSolver::projectLines ()
{
	Camera::CameraIntrinsic camera = glcanvas->getCamera()->getCameraParam();

	projectAllPoints (glcanvas->getCamera(), map, 640, 480);
//	projectAllPoints (position0, orientation0, camera, map, width, height);

	visibleLines.clear();

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

		Point2 PAim (
			PAcam.x()*camera.fx/PAcam.z() + camera.cx,
			PAcam.y()*camera.fy/PAcam.z() + camera.cy
		);
		Point2 PBim (
			PBcam.x()*camera.fx/PBcam.z() + camera.cx,
			PBcam.y()*camera.fy/PBcam.z() + camera.cy
		);

		// filter this line. We check for visibility in screen
		if (PAcam.z()<0 and PBcam.z()<0) {
			continue;
		}
		if (PAcam.z()>farPlane and PBcam.z()>farPlane) {
			continue;
		}
		if ((PAim.x()<0 or PAim.x()>width or PAim.y()<0 or PAim.y()>width) and
			(PBim.x()<0 or PBim.x()>width or PBim.y()<0 or PBim.y()>width)) {
			continue;
		}

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

	return;
}


pscalar PointSolver::LineSegment2D::error(Point2 &P)
{
	pscalar lsegmentsq = this->lengthSquared();
	Point2 Q = A.coord + (B.coord-A.coord)*((P-A.coord).dot(B.coord-A.coord)) / lsegmentsq;
	return (Q-P).dot(Q-P);
}


pscalar PointSolver::LineSegment2D::errorJacobian (Point2 &P, pscalar *jm)
{
	pscalar lsegmentsq = this->lengthSquared();

	pscalar dep1x, dep1y, dep2x, dep2y;
	pscalar p1x = A.coord.x(),
			p1y = A.coord.y(),
			p2x = B.coord.x(),
			p2y = B.coord.y(),
			px = P.x(), py = P.y();

	dep1x = -2*(p2y-p1y)*
		(p2x*py-p1x*py-p2y*px+p1y*px+p1x*p2y-p1y*p2x) *
		(p2y*py-p1y*py+p2x*px-p1x*px-p2y*p2y+p1y*p2y-p2x*p2x+p1x*p2x) /
		(lsegmentsq * lsegmentsq);
	dep1y = 2*(p2x-p1x)*
		(p2x*py-p1x*py-p2y*px+p1y*px+p1x*p2y-p1y*p2x) *
		(p2y*py-p1y*py+p2x*px-p1x*px-p2y*p2y+p1y*p2y-p2x*p2x+p1x*p2x) /
		(lsegmentsq * lsegmentsq);
	dep2x = 2*(p2y-p1y)*
		(p2x*py-p1x*py-p2y*px+p1y*px+p1x*p2y-p1y*p2x) *
		(p2y*py-p1y*py+p2x*px-p1x*px-p1y*p2y-p1x*p2x+p1y*p1y+p1x*p1x) /
		(lsegmentsq * lsegmentsq);
	dep2y = -2*(p2x-p1x)*
		(p2x*py-p1x*py-p2y*px+p1y*px+p1x*p2y-p1y*p2x)*
		(p2y*py-p1y*py+p2x*px-p1x*px-p1y*p2y-p1x*p2x+p1y*p1y+p1x*p1x) /
		(lsegmentsq * lsegmentsq);

	for (int i=0; i<7; i++) {
		jm[i] = dep1x*A.jacobian[i][0] + dep1y*A.jacobian[i][1] + dep2x*B.jacobian[i][0] + dep2y*B.jacobian[i][1];
	}
}


void PointSolver::prepareMatrices()
{
	Jac = Eigen::MatrixXd (imagePoints.size(), 7);
	Pcorrect = Eigen::VectorXd (7);
	pointErrs = Eigen::VectorXd (imagePoints.size());

	for (int ip=0; ip<imagePoints.size(); ip++) {
		pointErrs [ip] = imagePoints[ip].lineDistance;
		Point2 curpt (imagePoints[ip].px, imagePoints[ip].py);

		pscalar jacobianPt[7];

		LineSegment2D &line = visibleLines[imagePoints[ip].nearestLine];
		line.errorJacobian(curpt, jacobianPt);
		for (int n=0; n<7; n++) {
			Jac(ip, n) = jacobianPt[7];
		}
	}
}


void PointSolver::pairPointsWithLines()
{
	for (int ip=0; ip<imagePoints.size(); ip++) {
		ImagePoint &pixel = imagePoints[ip];

		Point2 P (pixel.px, pixel.py);
		pscalar cdist = 1e32;

		// find nearest line
		for (int il=0; il<visibleLines.size(); il++) {
			LineSegment2D &line = visibleLines[il];
			pscalar dist = line.error (P);
			if (dist < cdist) {
				pixel.nearestLine = il;
				pixel.lineDistance = dist;
			}
		}
	}
}
