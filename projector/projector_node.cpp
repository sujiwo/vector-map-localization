/*
 * map_points.cpp
 *
 *  Created on: Apr 11, 2015
 *      Author: sujiwo
 */


#include "SceneManager.h"
#include "PointListDrawObject.h"
#include "SignalsDrawObject.h"
#include "LineListDrawObject.h"
#include "LaneListDrawObject.h"
#include "CameraImageBackground.h"
#include "AreaListDrawObject.h"
#include "Rate.h"
#include <signal.h>
#include "vector_map.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>



VectorMap vectormap;
Eigen::Vector3f position;
Eigen::Quaternionf orientation;
tf::StampedTransform transform;


#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768
#define LINE_SCALE 0.05



void getTransform (Eigen::Quaternionf &ori, Point3 &pos)
{
	static tf::TransformListener listener;

	// target_frame    source_frame
//	listener.waitForTransform ("japan_7", "camera", ros::Time(), ros::Duration(10.0));
//	listener.lookupTransform ("japan_7", "camera", ros::Time(), transform);
	listener.waitForTransform ("camera1", "japan_7", ros::Time(), ros::Duration(10.0));
	listener.lookupTransform ("camera1", "japan_7", ros::Time(), transform);

	printf ("PosX:%f, PosY:%f, PosZ:%f\n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
//	tf::Vector3 &p = transform.getOrigin();
//	tf::Quaternion o = transform.getRotation();
//	pos.x()=p.x(); pos.y()=p.y(); pos.z()=p.z();
//	ori.w()=o.w(); ori.x()=o.x(); ori.y()=o.y(); ori.z()=o.z();
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
	SceneManager &scene = SceneManager::getInstance();
	scene.getCamera()->projectionMatrixFromCameraInfo (camInfoMsg);
	//debugMatrix (scene.getCamera()->getProjectionMatrix());
}


void interrupt (int s)
{
	exit(1);
}


int main (int argc, char **argv)
{
	SceneManager &app = SceneManager::getInstance(WINDOW_WIDTH, WINDOW_HEIGHT);
	vectormap.loadAll(argv[1]);

	// ROS business
	ros::init(argc, argv, "map_points", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;
	ros::Subscriber cameraInfoSubscriber = rosnode.subscribe ("/camera1/camera_info", 100, cameraInfoCallback);

	PointListDrawObject::Ptr pointList (new PointListDrawObject (vectormap));
	SignalsDrawObject::Ptr signals (new SignalsDrawObject (vectormap));
	LineListDrawObject::Ptr lineList (new LineListDrawObject (vectormap, LINE_SCALE));
	LaneListDrawObject::Ptr laneList (new LaneListDrawObject (vectormap, LINE_SCALE));
	CameraImageBackground::Ptr videoImg (new CameraImageBackground (rosnode, "/camera1/image_raw", WINDOW_WIDTH, WINDOW_HEIGHT));
	AreaListDrawObject::Ptr areaList (new AreaListDrawObject (vectormap));

	// Drawing order depend on this list
	app.addObject (videoImg);
	app.addObject (pointList);
	app.addObject (signals);
	app.addObject (lineList);
	app.addObject (laneList);
	app.addObject (areaList);

	pointList->setColor(255, 255, 0);
	//videoImg->setColor(0, 255, 255);

	signal (SIGINT, interrupt);

	Rate loop (25);
	while (true) {

		ros::spinOnce();

		try {
			getTransform (orientation, position);
		} catch (tf::TransformException &exc) {
		}

		//app.getCamera()->lookAt(orientation, position);
		//app.getCamera()->lookAt (transform);
		app.setPose (transform);

		app.update();
		loop.sleep();
	}

	return 0;
}
