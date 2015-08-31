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
#include "debug.h"


using std::string;
using std::cout;
using std::fstream;
const float farPlane = 50.0;


PointSolver::PointSolver (VectorMap *src, RenderWidget *gl, int w, int h) :
	map (src),
	glcanvas (gl),
	width (w), height (h)
{}


void PointSolver::solve (cv::Mat *processedInputImage, Point3 &startPosition, Quaternion &startOrientation)
{
	assert (processedInputImage->type()==CV_8UC1);
	image = processedInputImage;
	position0 = startPosition;
	orientation0 = startOrientation;

	projectLines ();
	debugProjection (visibleLines);
	prepareImage ();
	debugPointLinePair ();

	prepareMatrices ();
	solveForCorrection ();

	startPosition.x() -= Pcorrect[0];
	startPosition.y() -= Pcorrect[1];
	startPosition.z() -= Pcorrect[2];
	startOrientation.x() -= Pcorrect[3];
	startOrientation.y() -= Pcorrect[4];
	startOrientation.z() -= Pcorrect[5];
	startOrientation.w() -= Pcorrect[6];

	cout << Pcorrect << std::endl;
}


void PointSolver::debugProjection (vector<PointSolver::LineSegment2D> &projResult)
{
	fstream dumpline;
	dumpline.open ("/tmp/dumpline.txt", std::fstream::out);

	cv::Mat proj = cv::Mat::zeros (height, width, CV_8UC1);
	for (int i=0; i<projResult.size(); i++) {
		Point2 p1 = projResult[i].A.coord,
				p2 = projResult[i].B.coord;

		cv::line (proj, cv::Point2i(p1.x(),p1.y()), cv::Point2i(p2.x(),p2.y()), CV_RGB(255,255,255));
		dumpline << p1.x() << "," << p1.y() << "," << p2.x() << "," << p2.y() << "," << projResult[i].mapLid << std::endl;
	}

	cv::imwrite ("/tmp/debugprojection1.png", proj);

	dumpline << "# Orientation: " << orientation0.w() << "," << orientation0.x() << "," << orientation0.y() << "," << orientation0.z() << std::endl;
	dumpline << "# Position: " << position0.x() << "," << position0.y() << "," << position0.z() << std::endl;
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
	cv::Mat proj = cv::Mat::zeros (h, w, CV_8UC1);

	for(std::map<int, Line>::iterator it=map->lines.begin(); it!=map->lines.end(); it++) {
		Line l = it->second;

		Point3 A = map->getPoint (l.bpid),
			B = map->getPoint (l.fpid);
		int u1, v1, u2, v2;
		if (camera->project (A, u1, v1, w, h)==true and camera->project(B, u2, v2)==true) {
			Point2 pj1 (u1, h-v1), pj2 (u2, h-v2);
			cv::circle (proj, cv::Point2i(pj1.x(), pj1.y()), 2, 255);
			cv::circle (proj, cv::Point2i(pj2.x(), pj2.y()), 2, 255);
			cv::line (proj, cv::Point2i(pj1.x(),pj1.y()), cv::Point2i(pj2.x(),pj2.y()), CV_RGB(255,255,255));
		}
	}

	cv::imwrite ("/tmp/debugprojection2.png", proj);
}


bool projectPoint (Camera *camera, Point3 &src, Point3 &pointInCam, Point2 &res, int width, int height)
{
	pointInCam = camera->transform (src);
	int u, v;
	if (camera->project (src, u, v, width, height)==true) {
		res = Point2 (u, height-v);
		return true;
	}
	else return false;
}


void PointSolver::projectLines ()
{
	Camera *camera = glcanvas->getCamera();
	Camera::CameraIntrinsic &cameraParam = camera->getCameraParam();

//	projectAllPoints (glcanvas->getCamera(), map, width, height);

	visibleLines.clear();

	for(std::map<int, Line>::iterator it=map->lines.begin(); it!=map->lines.end(); it++) {
		Line l = it->second;

		Point3 A = map->getPoint (l.bpid),
			B = map->getPoint (l.fpid);

		Point3 PAcam, PBcam;
		Point2 PAim, PBim;

		if (projectPoint (camera, A, PAcam, PAim, width, height)==false or
			projectPoint (camera, B, PBcam, PBim, width, height)==false
		) {
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

		computeProjectionJacobian (position0, orientation0, A, PAcam, PAim,
				cameraParam.fx, cameraParam.fy, cameraParam.cx, cameraParam.cy,
				P1.jacobian);
		computeProjectionJacobian (position0, orientation0, B, PBcam, PBim,
				cameraParam.fx, cameraParam.fy, cameraParam.cx, cameraParam.cy,
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


void PointSolver::prepareMatrices ()
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
				// imagePoints.push_back (pt);

				Point2 P (pt.px, pt.py);
				pt.lineDistance = 1e32;

				// find nearest line
				for (int il=0; il<visibleLines.size(); il++) {
					LineSegment2D &line = visibleLines[il];
					pscalar dist = line.distanceSquared(P);
					if (dist < pt.lineDistance) {
						pt.nearestLine = il;
						pt.lineDistance = dist;
					}
				}

				if (pt.lineDistance < 10000.0) {
					imagePoints.push_back (pt);
				}
			}
		}
	}
}


void PointSolver::solveForCorrection ()
{
	Eigen::MatrixXd jat = Jac.transpose() * Jac;
	Eigen::VectorXd jae = Jac.transpose() * pointErrs;
	Pcorrect = jat.colPivHouseholderQr().solve (jae);
}


void PointSolver::debugPointLinePair ()
{
	fstream dumpPoint;
	dumpPoint.open ("/tmp/dumppoint.txt", fstream::out);

	for (int i=0; i<imagePoints.size(); i++) {
		ImagePoint &pixel = imagePoints[i];

		//debug ("%d,%f,%f,%d,%f", i, pixel.px, pixel.py, pixel.nearestLine, pixel.lineDistance);
		dumpPoint << i << "," << pixel.px << "," << pixel.py << "," << pixel.nearestLine << "," << pixel.lineDistance << std::endl;
	}

	dumpPoint.close ();
}


pscalar PointSolver::LineSegment2D::distanceSquared (Point2 &P)
{
	Vector2 M = B.coord - A.coord;
	pscalar t = M.dot(P-A.coord) / (M.squaredNorm());

	if (t<=0)
		return (P-A.coord).squaredNorm();
	else if (t>0 and t<1)
		return (P - (A.coord+M*t)).squaredNorm();
	else
		return (P - B.coord).squaredNorm();
}
