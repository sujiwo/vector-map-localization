/*
 * signals2.cpp
 *
 *  Created on: Apr 14, 2015
 *      Author: sujiwo
 */




#include <iostream>
#include <ros/ros.h>
#include "Camera.h"
#include "Rate.h"
#include "vector_map.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include "SignalsDrawObject.h"
#include <signal.h>
#include <vector>
#include "debug.h"



VectorMap vmap;
Camera camera;

Eigen::Vector3f position;
Eigen::Quaternionf orientation;



void getTransform (Eigen::Quaternionf &ori, Point3 &pos)
{
	static tf::TransformListener listener;
	tf::StampedTransform trf;

	// target_frame    source_frame
	listener.waitForTransform ("japan_7", "camera", ros::Time(), ros::Duration(10.0));
	listener.lookupTransform ("japan_7", "camera", ros::Time(), trf);

	tf::Vector3 &p = trf.getOrigin();
	tf::Quaternion o = trf.getRotation();
	pos.x()=p.x(); pos.y()=p.y(); pos.z()=p.z();
	ori.w()=o.w(); ori.x()=o.x(); ori.y()=o.y(); ori.z()=o.z();
}


void debugMatrix (const Matrix4 &prjm)
{
	std::cout << "###" << std::endl;
	for (int r=0; r<4; r++) {
		for (int c=0; c<4; c++) {
			std::cout << prjm(r, c) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "###" << std::endl;
}


void cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr camInfoMsg)
{
	camera.setSize (camInfoMsg->width, camInfoMsg->height);
	camera.projectionMatrixFromCameraInfo (camInfoMsg);
}


void interrupt (int s)
{
	exit(1);
}


void echoSignal (SignalsDrawObject &signObj)
{
	std::vector<SignalCircle> &signalList = signObj.getSignalList();
	int countSignal = 0;

	for (int i=0; i<signalList.size(); i++) {
		SignalCircle &signal = signalList[i];
		int u, v;
		float radiusInImagePlane;
		if (!signal.project(camera, u, v, radiusInImagePlane)) {
			continue;
		}
		else {
			countSignal += 1;
			debug ("%d,%d,%d,%f,%f,%f,%f",
				signal.id,
				u, v, radiusInImagePlane,
				signal.center.x(), signal.center.y(), signal.center.z());
		}
	}

	std::cout << "There are " << countSignal << " out of " << signalList.size() << std::endl;
}


int main (int argc, char **argv)
{
	// ROS business
	ros::init(argc, argv, "map_points", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;
	ros::Subscriber cameraInfoSubscriber = rosnode.subscribe ("/camera/camera_info", 100, cameraInfoCallback);

	vmap.loadAll ("data");
	SignalsDrawObject signals (vmap, false);

	signal (SIGINT, interrupt);

	Rate loop (25);
	while (true) {

		ros::spinOnce();

		try {
			getTransform (orientation, position);
		} catch (tf::TransformException &exc) {
		}

		camera.lookAt(orientation, position);
		echoSignal (signals);

		loop.sleep();
	}
}
