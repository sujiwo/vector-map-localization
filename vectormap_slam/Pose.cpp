/*
 * Pose.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sujiwo
 */

#include "Pose.h"
#include "tf_eigen.h"


void Pose::setRotationOffset (const Quaternion &rot)
{
	tf::Transform noff = eigen2tf (rot, Point3(0, 0, 0));
	tf::Transform coff = eigen2tf (orientationOffset, positionOffset);
	tf::Transform offset = noff * coff;
	tf2eigen (offset, orientationOffset, positionOffset);
}


void Pose::offset (double x, double y, double z, double roll, double pitch, double yaw)
{
	Point3 ofp (x, y, z);
	offset (ofp);
	offsetRoll (roll);
	offsetPitch (pitch);
	offsetYaw (yaw);
}


void Pose::offsetRoll(double roll)
{
	Quaternion rot (cos(roll/2), 0, 0, sin(roll/2));
	setRotationOffset (rot);
}


void Pose::offsetPitch(double pitch)
{
	Quaternion rot (cos(pitch/2), sin(pitch/2), 0, 0);
	setRotationOffset (rot);
}


void Pose::offsetYaw(double yaw)
{
	Quaternion rot (cos(yaw/2), 0, sin(yaw/2), 0);
	setRotationOffset (rot);
}


void Pose::set(const Point3 &newposition, const Quaternion &neworientation)
{
	position = newposition; orientation = neworientation;
}


void Pose::getAbsolute (Point3 &absPosition, Quaternion &absOrientation)
{
	Point3 newpos = -positionOffset;
	tf::Transform offset = eigen2tf (orientationOffset, newpos);
	tf::Transform curPose = eigen2tf (orientation, position);
	tf::Transform newPose = offset * curPose;

	tf2eigen (newPose, absOrientation, absPosition);
}


Vector3 Pose::getDirectionVector (bool useOffset)
{
	Point3 v1 = transformCameraToWorld (Point3 (0, 0, 0), useOffset);
	Point3 v2 = transformCameraToWorld (Point3 (0, 0, 1), useOffset);
	Vector3 v = v2 - v1;
	v.normalize();
	return v;
}


Vector3 Pose::getNormalVector (bool useOffset)
{
	Point3 v1 = transformCameraToWorld (Point3 (0, 0, 0), useOffset);
	Point3 v2 = transformCameraToWorld (Point3 (0, -1, 0), useOffset);
	Vector3 v = v2 - v1;
	v.normalize();
	return v;
}


Point3 Pose::getWorldPosition()
{
	return transformCameraToWorld (Point3 (0, 0, 0));
}


void Pose::fix()
{
	getAbsolute (position, orientation);
}


Point3 Pose::transformWorldToCamera (const Point3& pointInWorld, bool useOffset)
{
	Point3 cp; Quaternion co;

	if (useOffset)
		getAbsolute (cp, co);
	else {
		cp = position;
		co = orientation;
	}

	tf::Transform tfa = eigen2tf (co, cp);
	tf::Vector3 ptCam = tfa * eigen2tf (pointInWorld);
	return tf2eigen (ptCam);
}


Point3 Pose::transformCameraToWorld (const Point3& pointInCamera, bool useOffset)
{
	Point3 cp; Quaternion co;

	if (useOffset)
		getAbsolute (cp, co);
	else {
		cp = position;
		co = orientation;
	}

	tf::Transform tfa = eigen2tf (co, cp).inverse();
	tf::Vector3 ptCam = tfa * eigen2tf (pointInCamera);
	return tf2eigen (ptCam);
}


double Pose::getHeading()
{
	Vector3 dir = getDirectionVector ();
	dir.z() = 0;
	double d = acos (dir.y() / dir.squaredNorm());
	if (dir.x() < 0)
		return 2*M_PI - d;
	else return d;
}


/*
 * Negative angle returned by this function means that the vehicle is pitching down,
 * and vice versa.
 */
double Pose::getElevation()
{
	Vector3 dir = getDirectionVector ();
	dir.x() = 0;
	dir.normalize();
	double d = acos (dir.y() / dir.squaredNorm());
	if (dir.z() < 0)
		return -d;
	else return d;
}


double Pose::getBank ()
{
	Vector3 normal = getNormalVector ();
	normal.y() = 0;
	normal.normalize();
	double d = acos (normal.z() / normal.squaredNorm());
	if (normal.x() < 0)
		return -d;
	else return d;
}
