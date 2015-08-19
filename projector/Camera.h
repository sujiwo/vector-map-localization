/*
 * Camera.h
 *
 *  Created on: Apr 8, 2015
 *      Author: sujiwo
 */

#ifndef SRC_CAMERA_H_
#define SRC_CAMERA_H_


#include "Math.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>


class Camera {
public:
	Camera (int windowWidth=0, int windowHeight=0);
	virtual ~Camera();

	void setSize (int w, int h);

	inline int getWidth () { return imageWidth; }
	inline int getHeight () { return imageHeight; }

	inline Matrix4 &getViewMatrix()
	{ return viewMatrix; }

	inline Matrix4 &getProjectionMatrix()
	{ return projectionMatrix; }

	// These functions change View Matrix
	void lookAt (const Point3 &eyepos, const Point3 &centerOfView, const Eigen::Vector3f &up);
	void lookAt (const Quaternion &orientation, const Point3 &location);
	void lookAt (const tf::Transform &t);
	// ---

	// These functions change Projection Matrix
	void perspective (float fovy, float aspectRatio, float zNear, float zFar);
	void projectionMatrixFromCameraInfo (float fx, float fy, int imageWidth, int imageHeight, float cx, float cy);
	void projectionMatrixFromCameraInfo (sensor_msgs::CameraInfo::ConstPtr info);
	// ---

	// Playing with view & projection matrices
	inline Point3 project (const Point3 &pt)
	{
		Point4 p4 (pt.x(), pt.y(), pt.z(), 1);
		p4 = projectionMatrix * viewMatrix * p4;
		return Point3 (p4.x(), p4.y(), p4.z()) / p4.w();
	}

	bool project (const Point3 &pt, int &u, int &v, int reqImageWidth=0, int reqImageHeight=0);

	// Transform a point to camera coordinate
	inline Point3 transform (const Point3 &pt)
	{
		Point4 p4 (pt.x(), pt.y(), pt.z(), 1);
		p4 = viewMatrix  * p4;
		return Point3 (p4.x(), p4.y(), p4.z()) / p4.w();
	}

	inline static Point3 transform (const Point3 &pt, tf::StampedTransform &tfsource)
	{
		tf::Vector3 pt3 (pt.x(), pt.y(), pt.z());
		tf::Vector3 pt3s = tfsource * pt3;
		return Point3 (pt3s.x(), pt3s.y(), pt3s.z());
	}


	// directly mess up with matrices
	void setViewMatrix (const Matrix4 &vm)
	{ viewMatrix = vm; }

	void setProjectionMatrix (const Matrix4 &pm);


private:
	Point3 currentPosition;
	Quaternion currentOrientation;

	Matrix4 viewMatrix,
		projectionMatrix;

	int imageWidth, imageHeight;

	struct {
		float fovy, aspectRatio, zNear, zFar;
		bool isPerpective;
	} _perspective;
};

#endif /* SRC_CAMERA_H_ */
