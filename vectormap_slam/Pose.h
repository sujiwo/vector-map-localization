/*
 * Pose.h
 *
 *  Created on: Jul 28, 2015
 *      Author: sujiwo
 */

#ifndef _POSE_H_
#define _POSE_H_

#include "Math.h"


class Pose {
public:
	Pose() :
		position (0, 0, 0),
		orientation (Quaternion::Identity()),
		positionOffset (0, 0, 0),
		orientationOffset (Quaternion::Identity())
		{}

	void set (const Point3 &newposition, const Quaternion &neworientation);

	void offset (const Point3 &posOff, const Quaternion &oriOff=Quaternion::Identity())
	{
		positionOffset = posOff; orientationOffset = oriOff;
	}


	void offset (double x, double y, double z, double roll=0, double pitch=0, double yaw=0);

	Vector3 getDirectionVector (bool useOffset=true);
	Vector3 getNormalVector (bool useOffset=true);

	void offsetRoll (double roll);
	void offsetPitch (double pitch);
	void offsetYaw (double yaw);

	// combine transformation between fixed and offset to absolute
	void getAbsolute (Point3 &absPosition, Quaternion &absOrientation);
	// apply offset to fixed
	void fix ();

	void reset()
	{
		positionOffset = Point3 (0, 0, 0);
		orientationOffset = Quaternion::Identity();
	}

	void resetOrientationOffset ()
	{
		orientationOffset = Quaternion::Identity();
	}

	Point3& getPosition () { return position; }
	Quaternion& getOrientation () { return orientation; }
	Point3& getPositionOffset () { return positionOffset; }
	Quaternion& getOrientationOffset () { return orientationOffset; }

	Point3 getWorldPosition();

	Point3 transformWorldToCamera (const Point3& pointInWorld, bool useOffset=true);
	Point3 transformCameraToWorld (const Point3& pointInCamera, bool useOffset=true);

	double getHeading ();
	double getElevation ();
	double getBank ();

protected:
	// fixed
	Point3 position;
	Quaternion orientation;
	// offsets
	Point3 positionOffset;
	Quaternion orientationOffset;

	void setRotationOffset (const Quaternion &rot);
};

#endif /* _POSE_H_ */
