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

	position0 = startPos;
	orientation0 = startOrientation;
	image = inputImage;

	projectLines ();
	prepareImage ();
	prepareMatrices ();
	solveForCorrection ();
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


void computeProjectionJacobian (
	Point3 &t,			// Camera center coordinate
	Quaternion &q,		// Camera orientation
	Point3 &point,			// Original point position
	Point2 &pim,			// Point in image
	float fx,
	float fy,
	float cx,
	float cy,	// From Intrinsic Matrix
	pscalar jacobian[7][2]	// jacobian result
	)
{
	pscalar x = point.x(),
			y = point.y(),
			z = point.z();
	pscalar Z = (-2*q.y()*q.y()-2*q.x()*q.x()+1)*(z-t.z()) + (2*q.y()*q.z()+2*q.w()*q.x())*(y-t.y())+(2*q.x()*q.z()-2*q.w()*q.y())*(x-t.x());

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


void PointSolver2::projectLines()
{
	visibleLines.clear();
	Camera::CameraIntrinsic param = camera->getCameraParam();

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

		computeProjectionJacobian (position0, orientation0, line.p1, vLine.A.coord,
				param.fx, param.fy, param.cx, param.cy,
				P1.jacobian);
		computeProjectionJacobian (position0, orientation0, line.p2, vLine.B.coord,
				param.fx, param.fy, param.cx, param.cy,
				P2.jacobian);

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
	Jac = Eigen::MatrixXd::Zero (ipoints.size(), 7);
	Pcorrect = Eigen::VectorXd::Zero (7);
	pointErrs = Eigen::VectorXd::Zero (ipoints.size());

	for (int ip=0; ip<ipoints.size(); ip++) {
		pointErrs [ip] = ipoints[ip].lineDistance;
		Point2 curpt = ipoints[ip].coord;

		pscalar jacobianPt[7];

		LineSegment2D &line = visibleLines[ipoints[ip].nearestLine];
		line.errorJacobian(curpt, jacobianPt);
		for (int n=0; n<7; n++) {
			Jac(ip, n) = jacobianPt[n];
		}
		continue;
	}
	std::cout << Jac << std::endl;
}


pscalar LineSegment2D::distanceSquared (Point2 &P)
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


pscalar LineSegment2D::errorJacobian (Point2 &P, pscalar *jm)
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


void PointSolver2::solveForCorrection ()
{
	Eigen::MatrixXd jat = Jac.transpose() * Jac;
	Eigen::VectorXd jae = Jac.transpose() * pointErrs;
	Pcorrect = jat.colPivHouseholderQr().solve (jae);
//	std::cout << Pcorrect << std::endl;
}
