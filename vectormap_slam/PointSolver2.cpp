/*
 * PointSolver2.cpp
 *
 *  Created on: Sep 1, 2015
 *      Author: sujiwo
 */

#include "PointSolver2.h"
#include <cstdlib>


typedef cv::Vec3b Color3;


inline cv::Point2f tocv (Point2 &p)
{ return cv::Point2f (p.x(), p.y()); }


Point2 projectPoint (Point3 &src, Matrix4 &viewMatrix, PointSolver2::Projector &projectionMatrix)
{
	Point4 src4 (src.x(), src.y(), src.z(), 1);
	Point4 pcam = viewMatrix * src4;
	return projectionMatrix * pcam;
}


bool projectLine (Point3 &src1, Point3 &src2, Matrix4 &viewMatrix, PointSolver2::Projector &projectionMatrix, Point2 &lp1, Point2 &lp2)
{
	lp1 = projectPoint (src1, viewMatrix, projectionMatrix);
	lp2 = projectPoint (src2, viewMatrix, projectionMatrix);

	// decision for whether the projected points are outside
}


Matrix4 createViewMatrix (Point3 &cameraPosition, Quaternion &cameraOrientation)
{
	Matrix4 viewMatrix;
	viewMatrix.block<3,3>(0,0) = cameraOrientation.toRotationMatrix();
	viewMatrix.block<3,1>(0,3) = viewMatrix.block<3,3>(0,0) * (-cameraPosition);
	viewMatrix (3,3) = 1;
	return viewMatrix;
}


Matrix4 createViewMatrix (Point3 &cameraPosition, Point3 &centerOfView, Vector3 &_up)
{
	Matrix4 viewMatrix = Matrix4::Identity();
	Vector3 up = _up;

	Vector3 direction = centerOfView - cameraPosition;
	direction.normalize();

	up.normalize();
	Vector3 side = direction.cross(up);
	Vector3 Utrue = side.cross (direction);

	viewMatrix.block<1,3> (0, 0) = side;
	viewMatrix.block<1,3> (1, 0) = Utrue;
	viewMatrix.block<1,3> (2, 0) = -direction;
	viewMatrix.block<3,1> (0, 3) = -cameraPosition;
	return viewMatrix;
}


PointSolver2::PointSolver2 (vector<ModelLine> &modelIn, PointSolver2::Projector &proj) :
	projectionMatrix (proj),
	model(modelIn)
{}


void PointSolver2::solve (cv::Mat &inputImage, Point3 &startPos, Quaternion &startOrientation)
{
	assert (inputImage.type() == CV_8UC1);

	position0 = startPos;
	orientation0 = startOrientation;
	image = inputImage;
	currentViewMatrix = createViewMatrix (startPos, startOrientation);

	projectLines ();
	prepareImage ();
//	debugPointPairing ("/tmp/debugpoint.png");

	prepareMatrices ();
	solveForCorrection ();
}


void computeProjectionJacobian (
	Point3 &t,				// Camera center coordinate
	Quaternion &q,			// Camera orientation
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

	for (int lid=0; lid<model.size(); lid++) {
		ModelLine &line = model[lid];

		ProjectedPoint P1, P2;
		projectLine (line.p1, line.p2, currentViewMatrix, projectionMatrix, P1.coord, P2.coord);

		LineSegment2D vLine;
		vLine.A.coord = P1.coord, vLine.B.coord = P2.coord;
		vLine.modelLid = lid;

		computeProjectionJacobian (position0, orientation0, line.p1, vLine.A.coord,
				projectionMatrix.fx(), projectionMatrix.fy(), projectionMatrix.cx(), projectionMatrix.cy(),
				vLine.A.jacobian);
		computeProjectionJacobian (position0, orientation0, line.p2, vLine.B.coord,
				projectionMatrix.fx(), projectionMatrix.fy(), projectionMatrix.cx(), projectionMatrix.cy(),
				vLine.B.jacobian);

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


void PointSolver2::projectModel (cv::Mat &output, vector<ModelLine> &model, PointSolver2::Projector &projector, Matrix4 &viewMatrix)
{
	output = cv::Mat::zeros((int)projector.height, (int)projector.width, CV_8UC1);

	for (int lid=0; lid<model.size(); lid++) {
		ModelLine &line = model[lid];

		ProjectedPoint P1, P2;
		P1.coord = projectPoint (line.p1, viewMatrix, projector);
		P2.coord = projectPoint (line.p2, viewMatrix, projector);

		cv::Point p1v (P1.coord.x(), P1.coord.y());
		cv::Point p2v (P2.coord.x(), P2.coord.y());
		cv::circle (output, p1v, 3, 255);
		cv::circle (output, p2v, 3, 255);
		cv::line (output, p1v, p2v, 255);
	}
}


void PointSolver2::projectModel(cv::Mat &output, vector<ModelLine> &model, PointSolver2::Projector &projector, Point3 &cameraPosition, Quaternion &cameraOrientation)
{
	Matrix4 viewMatrix = createViewMatrix (cameraPosition, cameraOrientation);
	return PointSolver2::projectModel (output, model, projector, viewMatrix);
}


void PointSolver2::projectModel (cv::Mat &output, vector<ModelLine> &model, PointSolver2::Projector &projector, Point3 &cameraPosition, Point3 &centerOfView, Vector3 &_up)
{
	Matrix4 viewMatrix = createViewMatrix (cameraPosition, centerOfView, _up);
	return PointSolver2::projectModel (output, model, projector, viewMatrix);
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
//	std::cout << pointErrs << std::endl;
//	std::cout << Jac << std::endl;
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


Point2 LineSegment2D::nearestTo (const Point2 &P)
{
	Vector2 M = B.coord - A.coord;
	pscalar t = M.dot(P-A.coord) / (M.squaredNorm());

	if (t<=0)
		return A.coord;
	else if (t>0 and t<1)
		return A.coord + M*t;
	else return B.coord;
}


void LineSegment2D::errorJacobian1 (
	const float &px,
	const float &py,
	const float &p1x,
	const float &p1y,
	const float &p2x,
	const float &p2y,
	const float lsegmentsq,
	float &dep1x, float &dep1y, float &dep2x, float &dep2y)
{
	dep1x = -2 * (px - p1x);
	dep1y = -2 * (py - p1y);
	dep2x = 0;
	dep2y = 0;
}


void LineSegment2D::errorJacobian2 (
		const float &px,
		const float &py,
		const float &p1x,
		const float &p1y,
		const float &p2x,
		const float &p2y,
		const float lsegmentsq,
		float &dep1x, float &dep1y, float &dep2x, float &dep2y)
{
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
}


void LineSegment2D::errorJacobian3 (
	const float &px,
	const float &py,
	const float &p1x,
	const float &p1y,
	const float &p2x,
	const float &p2y,
	const float lsegmentsq,
	float &dep1x, float &dep1y, float &dep2x, float &dep2y)
{
	dep1x = 0;
	dep1y = 0;
	dep2x = -2 * (px - p2x);
	dep2y = -2 * (py - p2y);
}


void LineSegment2D::errorJacobian (Point2 &P, pscalar *jm)
{
	Vector2 M = B.coord - A.coord;
	pscalar t = M.dot(P-A.coord) / (M.squaredNorm());

	pscalar lsegmentsq = this->lengthSquared();

	pscalar dep1x, dep1y, dep2x, dep2y;
	pscalar p1x = A.coord.x(),
			p1y = A.coord.y(),
			p2x = B.coord.x(),
			p2y = B.coord.y(),
			px = P.x(), py = P.y();

	if (t<=0)
		errorJacobian1 (px, py, p1x, p1y, p2x, p2y, lsegmentsq, dep1x, dep1y, dep2x, dep2y);
	else if (t>0 and t<1)
		errorJacobian2 (px, py, p1x, p1y, p2x, p2y, lsegmentsq, dep1x, dep1y, dep2x, dep2y);
	else
		errorJacobian3 (px, py, p1x, p1y, p2x, p2y, lsegmentsq, dep1x, dep1y, dep2x, dep2y);

	for (int i=0; i<7; i++) {
		jm[i] = dep1x*A.jacobian[i][0] + dep1y*A.jacobian[i][1] + dep2x*B.jacobian[i][0] + dep2y*B.jacobian[i][1];
	}
}


void PointSolver2::solveForCorrection ()
{
	Eigen::MatrixXd jat = Jac.transpose() * Jac;
	Eigen::VectorXd jae = Jac.transpose() * pointErrs;
	Pcorrect = jat.colPivHouseholderQr().solve (jae);
	std::cout << jat << std::endl;
}


PointSolver2::Projector::Projector (pscalar fx, pscalar fy, pscalar cx, pscalar cy)
{
	matrix = Eigen::Matrix<pscalar,3,4>::Zero ();
	matrix(0, 0) = fx;
	matrix(0, 2) = cx;
	matrix(1, 1) = fy;
	matrix(1, 2) = cy;
	matrix(2, 2) = 1;
}


PointSolver2::Projector::Projector (pscalar angleDegree, int w, int h):
	width (w), height(h)
{
	matrix = Eigen::Matrix<pscalar,3,4>::Zero ();
	pscalar angle = (angleDegree/2) * M_PI/180.0;
	pscalar cx = width / 2;
	pscalar cy = height / 2;
	pscalar fx = w / (2*tan(angle));
	pscalar fy = fx;
	//Projector (-fx, fy, cx, cy);

	matrix(0, 0) = -fx;
	matrix(0, 2) = cx;
	matrix(1, 1) = fy;
	matrix(1, 2) = cy;
	matrix(2, 2) = 1;
}


Point2 PointSolver2::Projector::operator *(Point4 &pointCam)
{
	Point3 phom = this->matrix * pointCam;
	return Point2 (phom.x(), phom.y()) / phom.z();
}


void poseFromViewMatrix (Matrix4 &viewMatrix, Point3 &position, Quaternion &orientation)
{
	Matrix3 rotMat = viewMatrix.block<3,3> (0, 0);
	orientation = Quaternion (rotMat);
	position = -rotMat.transpose() * viewMatrix.block<3,1> (0, 3);
}


void PointSolver2::debugPointPairing(const char *imgfilename)
{
	cv::Mat dbgImage;
	dbgImage = cv::Mat::zeros (image.rows, image.cols, CV_8UC3);
	Color3 white (255,255,255);

	for (auto &line: visibleLines) {
		cv::Point2f p1 = tocv (line.A.coord), p2 = tocv (line.B.coord);
		cv::line(dbgImage, p1, p2, CV_RGB(120, 120, 120));
	}

	for (auto &p: ipoints) {
		cv::Point2f pos = tocv (p.coord);
		LineSegment2D &line = visibleLines[p.nearestLine];
		Point2 lnear = line.nearestTo(p.coord);
		cv::line (dbgImage, pos, tocv(lnear), CV_RGB(0, 80, 0));
		dbgImage.at<Color3> (pos) = white;
	}

	cv::imwrite (imgfilename, dbgImage);
}
