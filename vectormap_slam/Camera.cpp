/*
 * Camera.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: sujiwo
 */

#include "Camera.h"


#define NEAR_PLANE 1.0
#define FAR_PLANE 40.0


Camera::Camera (int w, int h) :
	imageWidth(w), imageHeight(h)
{
	_perspective.isPerpective = false;
}


Camera::~Camera() {
	// TODO Auto-generated destructor stub
}


void Camera::lookAt (const Point3 &eyePos, const Point3 &centerOfView, const Vector3 &_up)
{
	viewMatrix = Matrix4::Identity();
	Vector3 up = _up;

	Vector3 direction;
	direction.x() = centerOfView.x() - eyePos.x();
	direction.y() = centerOfView.y() - eyePos.y();
	direction.z() = centerOfView.z() - eyePos.z();
	direction.normalize();

	up.normalize();
	Vector3 side = direction.cross(up);
	Vector3 Utrue = side.cross (direction);

	viewMatrix (0, 0) = side.x();
	viewMatrix (0, 1) = side.y();
	viewMatrix (0, 2) = side.z();
	viewMatrix (1, 0) = Utrue.x();
	viewMatrix (1, 1) = Utrue.y();
	viewMatrix (1, 2) = Utrue.z();
	viewMatrix (2, 0) = -direction.x();
	viewMatrix (2, 1) = -direction.y();
	viewMatrix (2, 2) = -direction.z();

	viewMatrix (0, 3) = -eyePos.x();
	viewMatrix (1, 3) = -eyePos.y();
	viewMatrix (2, 3) = -eyePos.z();
}


/* this is a working version */
void Camera::lookAt (const Quaternion &orientation, const Point3 &position)
{
	viewMatrix = Matrix4::Identity();
	Quaternion orin;
	orin = orientation * Eigen::Quaternionf( Eigen::AngleAxisf(degreeToRadian(180), Eigen::Vector3f::UnitX()) );
	orin.normalize();

	viewMatrix.block<3,3> (0,0) = orin.toRotationMatrix().transpose();
	viewMatrix.block<3,1> (0,3) = viewMatrix.block<3,3> (0,0) * (-position);

	currentPosition = position;
	currentOrientation = orientation;
}


void Camera::lookAt (const tf::Transform &tnf)
{
	tfScalar mrt[16];
	tnf.getOpenGLMatrix (mrt);
	// Column-major
	viewMatrix(0, 0) = mrt[0];
	viewMatrix(1, 0) = mrt[1];
	viewMatrix(2, 0) = mrt[2];
	viewMatrix(3, 0) = mrt[3];
	viewMatrix(0, 1) = mrt[4];
	viewMatrix(1, 1) = mrt[5];
	viewMatrix(2, 1) = mrt[6];
	viewMatrix(3, 1) = mrt[7];
	viewMatrix(0, 2) = mrt[8];
	viewMatrix(1, 2) = mrt[9];
	viewMatrix(2, 2) = mrt[10];
	viewMatrix(3, 2) = mrt[11];
	viewMatrix(0, 3) = mrt[12];
	viewMatrix(1, 3) = mrt[13];
	viewMatrix(2, 3) = mrt[14];
	viewMatrix(3, 3) = mrt[15];

	// Rotate 180 around Z-axis and Y-axis
	Eigen::Matrix4f rotfix = Eigen::Matrix4f::Identity();
	rotfix (1,1) = -1;
	rotfix (2,2) = -1;
	rotfix (3,3) = 1;
	viewMatrix = rotfix * viewMatrix;
}


void Camera::perspective (float fovy, float aspectRatio, float zNear, float zFar)
{
	_perspective.isPerpective = true;
	_perspective.fovy = fovy;
	_perspective.aspectRatio = aspectRatio;
	_perspective.zNear = zNear;
	_perspective.zFar = zFar;

	projectionMatrix = Matrix4::Zero();
	double angle = degreeToRadian(fovy/2);
	double f = 1 / (tan(angle));
	projectionMatrix (0, 0) = f/aspectRatio;
	projectionMatrix (1, 1) = f;
	projectionMatrix (2, 2) = (zFar + zNear) / (zNear - zFar);
	projectionMatrix (3, 2) = -1;
	projectionMatrix (2, 3) = 2 * zFar * zNear / (zNear - zFar);
}


void Camera::projectionMatrixFromCameraInfo (float fx, float fy, int w, int h, float cx, float cy)
{
	_perspective.isPerpective = false;

	projectionMatrix = Eigen::Matrix4f::Zero();

	double far_plane = FAR_PLANE, near_plane = NEAR_PLANE;
//	double zoom_x = (float)imageWidth / (float)w,
//			zoom_y = (float)imageHeight / (float)h;
	double zoom_x = 1.0, zoom_y = 1.0;

	projectionMatrix(0, 0) = 2.0 * fx/w * zoom_x;
	projectionMatrix(1, 1) = 2.0 * fy/h * zoom_y;
	projectionMatrix(0, 2) = 2.0 * (0.5 - cx/w) * zoom_x;
	projectionMatrix(1, 2) = 2.0 * (cy/h - 0.5) * zoom_y;
	projectionMatrix(2, 2) = -(far_plane+near_plane) / (far_plane-near_plane);
	projectionMatrix(2, 3) = -2.0*far_plane*near_plane / (far_plane-near_plane);
	projectionMatrix(3, 2) = -1;
}


void Camera::projectionMatrixFromCameraInfo (sensor_msgs::CameraInfo::ConstPtr info)
{
	float
		fx = info->P[0],
		fy = info->P[5],
		w = info->width,
		h = info->height,
		cx = info->P[2],
		cy = info->P[6];

	return projectionMatrixFromCameraInfo (fx, fy, w, h, cx, cy);
}


bool Camera::project (const Point3 &pt, int &u, int &v, int reqImageWidth, int reqImageHeight)
{
	if (reqImageWidth==0 or reqImageHeight==0) {
		reqImageWidth = imageWidth;
		reqImageHeight = imageHeight;
	}

	Point3 prj = project (pt);

	// cull point
	if (fabs(prj.x())>1 or fabs(prj.y())>1 or fabs(prj.z())>1) {
		u = -1, v = -1;
		return false;
	}

	prj.x() = (prj.x() + 1.0) / 2;
	prj.y() = (prj.y() + 1.0) / 2;

	float up = (float)reqImageWidth * prj.x(),
		vp = (float)reqImageHeight * prj.y();

	u = (int)up, v = (int)vp;
	return true;
}


void Camera::setSize(int w, int h)
{
	imageWidth=w, imageHeight=h;
	if (_perspective.isPerpective==true) {
		_perspective.aspectRatio = (float)w / (float)h;
		perspective (_perspective.fovy, _perspective.aspectRatio, _perspective.zNear, _perspective.zFar);
	}
}
