/*
 * PointSolver2.cpp
 *
 *  Created on: Sep 1, 2015
 *      Author: sujiwo
 */

#include "PointSolver2.h"
#include <cstdlib>
#include <cmath>


typedef cv::Vec3b Color3;


const pscalar nearPlane = 0.1,
	farPlane = 100.0;


inline cv::Point2f tocv (Point2 &p)
{ return cv::Point2f (p.x(), p.y()); }


Point2 projectPoint (Point3 &src, Matrix4 &viewMatrix, PointSolver2::Projector &projectionMatrix, Point3 &pointInCam)
{
	Point4 src4 (src.x(), src.y(), src.z(), 1);
	Point4 pcam = viewMatrix * src4;
	pointInCam = pcam.head(3);
	return projectionMatrix * pcam;
}


void projectLine (Point3 &src1, Point3 &src2, Matrix4 &viewMatrix, PointSolver2::Projector &projectionMatrix, Point2 &lp1, Point2 &lp2, Point3 &lp1InCam, Point3 &lp2InCam)
{
	lp1 = projectPoint (src1, viewMatrix, projectionMatrix, lp1InCam);
	lp2 = projectPoint (src2, viewMatrix, projectionMatrix, lp2InCam);
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

	prepareImage ();

#define ITERATION 5

	for (int np=0; np<ITERATION; np++) {
		currentViewMatrix = createViewMatrix (position0, orientation0);

		projectLines ();
		findNearestLinePerPoint ();
		prepareMatrices ();
		solveForCorrection ();
	}
//	currentViewMatrix = createViewMatrix (position0, orientation0);
	debugDraw ("/tmp/draw.png", &position0, &orientation0);

	return;
}


//void computeProjectionJacobian (
//	Point3 &t,				// Camera center coordinate
//	Quaternion &q,			// Camera orientation
//	Point3 &point,			// Original point position
//	Point2 &pim,			// Point in image
//	float fx,
//	float fy,
//	float cx,
//	float cy,	// From Intrinsic Matrix
//	pscalar jacobian[7][2]	// jacobian result
//	)
//{
//	pscalar x = point.x(),
//			y = point.y(),
//			z = point.z();
//	pscalar Z = (-2*q.y()*q.y()-2*q.x()*q.x()+1)*(z-t.z()) + (2*q.y()*q.z()+2*q.w()*q.x())*(y-t.y())+(2*q.x()*q.z()-2*q.w()*q.y())*(x-t.x());
//
//	pscalar
//		Tx = 2 * (q.w()*q.y() - q.x()*q.z()),
//		Ty = -2 * (q.y()*q.z() + q.w()*q.x()),
//		Tz = 2*q.y()*q.y() + 2*q.x()*q.x() - 1,
//		Qx = -4*q.x()*(z-t.z()) + 2*q.w()*(y-t.y()) + 2*q.z()*(x-t.x()),
//		Qy = -4*q.y()*(z-t.z()) + 2*q.z()*(y-t.y()) - 2*q.w()*(x-t.x()),
//		Qz = 2*q.y()*(y - t.y()) + 2*q.x()*(x - t.x()),
//		Qw = 2*q.x()*(y - t.y()) - 2*q.y()*(x - t.x());
//
//	// dtxx
//	jacobian[0][0] = ((fx*(2*q.z()*q.z() + 2*q.y()*q.y()-1) + cx*Tx) / Z) - Tx*pim.x()/Z;
//	// dtxy
//	jacobian[0][1] = ((cy*Tx + fy*(-2*q.w()*q.z()-2*q.x()*q.y())) / Z) - Tx*pim.y()/Z;
//	// dtyx
//	jacobian[1][0] = ((cx*Ty+fx*(2*q.w()*q.z()-2*q.x()*q.y())) / Z) - Ty*pim.x()/Z;
//	// dtyy
//	jacobian[1][1] = ((fy*(2*q.z()*q.z() + 2*q.x()*q.x() - 1) + cy*Ty) / Z) - Ty*pim.y()/Z;
//	// dtzx
//	jacobian[2][0] = ((fx*(-2*q.x()*q.z()-2*q.w()*q.y()) + cx*Tz) / Z) - ((Tz*pim.x())/Z);
//	// dtzy
//	jacobian[2][1] = ((fx*(-2*q.x()*q.z()-2*q.w()*q.y()) + cx*Tz) / Z) - ((Tz*pim.y())/Z);
//	// dqxx
//	jacobian[3][0] = ((fx*(2*q.z()*(z-t.z()) + 2*q.y()*(y-t.y())) + cx*Qx)/Z) - (Qx*pim.x()/Z);
//	// dqxy
//	jacobian[3][1] = ((cy*Qx + fy*(-2*q.w()*(z-t.z()) -4*q.x()*(y-t.y()) + 2*q.y()*(x-t.x())) )/Z) - (Qx*pim.y()/Z);
//	// dqyx
//	jacobian[4][0] = (cx*Qy + fx*(2*q.w()*(z-t.z()) +2*q.x()*(y-t.y()) -4*q.y()*(x-t.x())))/Z  -  Qy*pim.x()/Z;
//	// dqyy
//	jacobian[4][1] = (fy*(2*q.z()*(z-t.z()) + 2*q.x()*(x-t.x())) + cy*Qy)/Z  -  Qy*pim.y()/Z;
//	// dqzx
//	jacobian[5][0] = (fx*(2*q.x()*(z-t.z()) - 2*q.w()*(y-t.y()) - 4*q.z()*(x-t.x())) + cx*Qz)/Z - Qz*pim.x()/Z;
//	// dqzy
//	jacobian[5][1] = (fy*(2*q.y()*(z-t.z()) - 4*q.z()*(y-t.y()) + 2*q.w()*(x-t.x())) + cy*Qz)/Z - Qz*pim.y()/Z;
//	// dqwx
//	jacobian[6][0] = (fx*(2*q.y()*(z-t.z()) - 2*q.z()*(y-t.y())) + cx*Qw)/Z - Qw*pim.x()/Z;
//	// dqwy
//	jacobian[6][1] = (fy*(2*q.z()*(x-t.x()) - 2*q.x()*(z-t.z())) + cy*Qw)/Z - Qw*pim.y()/Z;
//}
void computeProjectionJacobian (
	Point3 &t,				// Camera center coordinate
	Quaternion &q,			// Camera orientation
	Point3 &point,			// Original point position
	Point3 &pcam,			// point position in camera coordinate system
	Point2 &pim,			// Point in image
	const Matrix4 &projectionMatrix,
	pscalar jacobian[7][2]	// jacobian result
	)
{
	auto
		xs = point.x(),
		ys = point.y(),
		zs = point.z();
	auto
		x = pcam.x(),
		y = pcam.y(),
		z = pcam.z();
	auto
		qx = q.x(),
		qy = q.y(),
		qz = q.z(),
		qw = q.w();
	auto
		tx = t.x(),
		ty = t.y(),
		tz = t.z();

	auto
		fx = projectionMatrix(0, 0),
		fy = projectionMatrix(1, 1),
		cx = projectionMatrix(0, 2),
		cy = projectionMatrix(1, 2),
		a = projectionMatrix(2, 2),
		b = projectionMatrix(2, 3);

	auto
		dxtx = 2*qz*qz + 2*qy*qy - 1,
		dxty = 2*qw*qz - 2*qx*qy,
		dxtz = -2*qx*qz - 2*qw*qy,
		dxqx = 2*qz*(zs-tz) + 2*qy*(ys-ty),
		dxqy = 2*qw*(zs-tz) + 2*qx*(ys-ty) - 4*qy*(xs-tx),
		dxqz = 2*qx*(zs-tz) - 2*qw*(ys-ty) - 4*qz*(xs-tx),
		dxqw = 2*qy*(zs-tz) - 2*qz*(ys-ty);

	auto
		dytx = -2*qw*qz - 2*qx*qy,
		dyty = 2*qz*qz + 2*qx*qx - 1,
		dytz = 2*qw*qx - 2*qy*qz,
		dyqx = -2*qw*(zs-tz) - 4*qx*(ys-ty) + 2*qy*(xs-tx),
		dyqy = 2*qz*(zs-tz) + 2*qx*(xs-tx),
		dyqz = 2*qy*(zs-tz) - 4*qz*(ys-ty)+2*qw*(xs-tx),
		dyqw = 2*qz*(xs-tx) - 2*qx*(zs-tz);

	auto
		dztx = 2*qw*qy - 2*qx*qz,
		dzty = -2*qy*qz - 2*qw*qx,
		dztz = 2*qy*qy + 2*qx*qx - 1,
		dzqx = -4*qx*(zs-tz) + 2*qw*(ys-ty) + 2*qz*(xs-tx),
		dzqy = -4*qy*(zs-tz) + 2*qz*(ys-ty) - 2*qw*(xs-tx),
		dzqz = 2*qy*(ys-ty) + 2*qx*(xs-tx),
		dzqw = 2*qx*(ys-ty) - 2*qy*(xs-tx);

	auto quotient = a*z + b;

	jacobian [0][0] = (cx*dztx + fx*dxtx)/quotient - (a*(cx*z + fx*x)*dztx)/(quotient*quotient);
	jacobian [0][1] = (cy*dztx + fy*dytx)/quotient - (a*(cy*z + fy*y)*dztx)/(quotient*quotient);

	jacobian [1][0] = (cx*dzty + fx*dxty)/quotient - (a*(cx*z + fx*x)*dzty)/(quotient*quotient);
	jacobian [1][1] = (cy*dzty + fy*dyty)/quotient - (a*(cy*z + fy*y)*dzty)/(quotient*quotient);

	jacobian [2][0] = (cx*dztz + fx*dxtz)/quotient - (a*(cx*z + fx*x)*dztz)/(quotient*quotient);
	jacobian [2][1] = (cy*dztz + fy*dytz)/quotient - (a*(cy*z + fy*y)*dztz)/(quotient*quotient);

	jacobian [3][0] = (cx*dzqx + fx*dxqx)/quotient - (a*(cx*z + fx*x)*dzqx)/(quotient*quotient);
	jacobian [3][1] = (cy*dzqx + fy*dyqx)/quotient - (a*(cy*z + fy*y)*dzqx)/(quotient*quotient);

	jacobian [4][0] = (cx*dzqy + fx*dxqy)/quotient - (a*(cx*z + fx*x)*dzqy)/(quotient*quotient);
	jacobian [4][1] = (cy*dzqy + fy*dyqy)/quotient - (a*(cy*z + fy*y)*dzqy)/(quotient*quotient);

	jacobian [5][0] = (cx*dzqz + fx*dxqz)/quotient - (a*(cx*z + fx*x)*dzqz)/(quotient*quotient);
	jacobian [5][1] = (cy*dzqz + fy*dyqz)/quotient - (a*(cy*z + fy*y)*dzqz)/(quotient*quotient);

	jacobian [6][0] = (cx*dzqw + fx*dxqw)/quotient - (a*(cx*z + fx*x)*dzqw)/(quotient*quotient);
	jacobian [6][1] = (cy*dzqw + fy*dyqw)/quotient - (a*(cy*z + fy*y)*dzqw)/(quotient*quotient);

}


void PointSolver2::projectLines()
{
	visibleLines.clear();

	for (int lid=0; lid<model.size(); lid++) {
		ModelLine &line = model[lid];

		ProjectedPoint P1, P2;
		projectLine (line.p1, line.p2, currentViewMatrix, projectionMatrix, P1.coord, P2.coord, P1.inCam, P2.inCam);

		LineSegment2D vLine (P1, P2, lid);

		computeProjectionJacobian (
				position0,
				orientation0,
				line.p1,
				vLine.A.inCam,
				vLine.A.coord,
				projectionMatrix.matrix,
				vLine.A.jacobian);
		computeProjectionJacobian (
				position0,
				orientation0,
				line.p2,
				vLine.B.inCam,
				vLine.B.coord,
				projectionMatrix.matrix,
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
				pt.coord = projectionMatrix.ScreenToNormalized(j, i);
				pt.nearestLine = -1;
				pt.lineDistance = -1;

				pt.lineDistance = 1e32;
				ipoints.push_back (pt);

			}
		}
	}
}


void PointSolver2::findNearestLinePerPoint ()
{
	for (auto &point: ipoints) {
		point.lineDistance = 1e32;
		for (int il=0; il<visibleLines.size(); il++) {
			LineSegment2D &line = visibleLines[il];
			pscalar dist = line.distance(point.coord);
			if (fabs(dist) < fabs(point.lineDistance)) {
				point.nearestLine = il;
				point.lineDistance = dist;
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
		P1.coord = projectPoint (line.p1, viewMatrix, projector, P1.inCam);
		P2.coord = projectPoint (line.p2, viewMatrix, projector, P2.inCam);

		Point2 P1scr = projector.NormalizedToScreen(P1.coord),
			P2scr = projector.NormalizedToScreen(P2.coord);

		cv::Point p1v (P1scr.x(), P1scr.y());
		cv::Point p2v (P2scr.x(), P2scr.y());
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


void LineSegment2D::errorJacobian (Point2 &P, pscalar *jm)
{
	for (int i=0; i<7; i++) {
		jm[i] = sin*A.jacobian[i][0] - cos*B.jacobian[i][1];
	}
}


void PointSolver2::solveForCorrection ()
{
	Eigen::MatrixXd jat = Jac.transpose() * Jac;
	Eigen::VectorXd jae = Jac.transpose() * pointErrs;
	Pcorrect = jat.colPivHouseholderQr().solve (jae);
	position0.x() -= Pcorrect[0];
	position0.y() -= Pcorrect[1];
	position0.z() -= Pcorrect[2];
	orientation0.x() -= Pcorrect[3];
	orientation0.y() -= Pcorrect[4];
	orientation0.z() -= Pcorrect[5];
	orientation0.w() -= Pcorrect[6];
//	orientation0.normalize();
//	std::cout << jat << std::endl;
//	std::cout << Pcorrect << std::endl;
}


PointSolver2::Projector::Projector (pscalar fx, pscalar fy, pscalar cx, pscalar cy, int _w, int _h) :
	width(_w), height(_h)
{
	matrix = Matrix4::Zero ();
	matrix(0, 0) = 2*fx/width;
	matrix(0, 2) = 2.0*(0.5 - cx/width);
	matrix(1, 1) = 2.0*fy/height;
	matrix(1, 2) = 2.0*(cy/height - 0.5);
	matrix(2, 2) = -(farPlane+nearPlane) / (farPlane-nearPlane);
	matrix(2, 3) = -2*.0*farPlane*nearPlane / (farPlane-nearPlane);
	matrix(3, 2) = -1;
}


PointSolver2::Projector::Projector (pscalar angleDegree, int w, int h):
	width (w), height(h)
{
	matrix = Matrix4::Zero ();
	pscalar aspectRatio = width / height;
	pscalar angle = (angleDegree/2) * M_PI/180.0;
	pscalar d = 1 / tan(angle);

	//Projector (-fx, fy, cx, cy);
	matrix (0, 0) = d/aspectRatio;
	matrix (1, 1) = d;
	matrix (2, 2) = (nearPlane+farPlane) / (nearPlane - farPlane);
	matrix (2, 3) = 2*nearPlane*farPlane / (nearPlane - farPlane);
	matrix (3, 2) = -1;
}


Point2 PointSolver2::Projector::operator *(Point4 &pointCam)
{
	Point4 ptmp = this->matrix * pointCam;
	Point3 phom = ptmp.head<3>() / ptmp(3);

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
		Point2 pA = projectionMatrix.NormalizedToScreen(line.A.coord),
			pB = projectionMatrix.NormalizedToScreen(line.B.coord);
		cv::Point2f p1 = tocv (pA), p2 = tocv (pB);
		cv::line(dbgImage, p1, p2, CV_RGB(120, 120, 120));
	}

	for (auto &p: ipoints) {
		Point2 ps = projectionMatrix.NormalizedToScreen(p.coord);
		cv::Point2f pos = tocv (ps);
		LineSegment2D &line = visibleLines[p.nearestLine];
		Point2 lnear = line.nearestTo(p.coord);
		Point2 lnearsc = projectionMatrix.NormalizedToScreen(lnear);
		cv::line (dbgImage, pos, tocv(lnearsc), CV_RGB(0, 80, 0));
		dbgImage.at<Color3> (pos) = white;
	}

	cv::imwrite (imgfilename, dbgImage);
}


void PointSolver2::debugDraw (const char *imgname, Point3 *pos, Quaternion *orin)
{
	cv::Mat image;
//	Matrix4 currentViewMatrix = createViewMatrix (position0, orientation0);
	Matrix4 cViewMatrix;
	if (pos==NULL)
		cViewMatrix = currentViewMatrix;
	else
		cViewMatrix = createViewMatrix (*pos, *orin);
	PointSolver2::projectModel (image, model, this->projectionMatrix, cViewMatrix);
	cv::imwrite (imgname, image);
}
