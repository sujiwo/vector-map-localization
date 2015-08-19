/*
 * tf_eigen.h
 *
 *  Created on: Jul 18, 2015
 *      Author: sujiwo
 */

#ifndef TF_EIGEN_H_
#define TF_EIGEN_H_


#include <tf/tf.h>
#include "Math.h"


inline void tf2eigen (const tf::Transform &transform, Eigen::Quaternionf &orientation, Point3 &position)
{
	position.x()=transform.getOrigin().x();
	position.y()=transform.getOrigin().y();
	position.z()=transform.getOrigin().z();
	orientation.w()=transform.getRotation().w();
	orientation.x()=transform.getRotation().x();
	orientation.y()=transform.getRotation().y();
	orientation.z()=transform.getRotation().z();
}


inline tf::Transform eigen2tf (const Eigen::Quaternionf &orientation, const Point3 &position)
{
	tf::Transform trf;
	tf::Vector3 p (position.x(), position.y(), position.z());
	tf::Quaternion o (orientation.x(), orientation.y(), orientation.z(), orientation.w());

	trf.setOrigin(p);
	trf.setRotation(o);
	return trf;
}


inline tf::Vector3 eigen2tf (const Vector3 &v)
{
	return tf::Vector3 (v.x(), v.y(), v.z());
}


inline Vector3 tf2eigen (const tf::Vector3 &v)
{
	return Vector3 (v.x(), v.y(), v.z());
}


inline void getRPY (const Eigen::Quaternionf &rot, double &roll, double &pitch, double &yaw)
{
	// Test Quaternion <==> RPY
	tf::Quaternion o (rot.x(), rot.y(), rot.z(), rot.w());
	tf::Matrix3x3 mat3x3 (o);
	mat3x3.getRPY (roll, pitch, yaw);
}


inline Eigen::Quaternionf getQuaternion (const double &roll, const double &pitch, double &yaw)
{
	tf::Matrix3x3 mat33;
	mat33.setRPY(roll, pitch, yaw);
	tf::Quaternion q;
	mat33.getRotation(q);

	return Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
}

#endif /* TF_EIGEN_H_ */
