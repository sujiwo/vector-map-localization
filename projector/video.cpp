/*
 * rectangle.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: sujiwo
 */

#include "SceneManager.h"
#include "CameraImageBackground.h"
#include "Rate.h"
#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


#define WIDTH 1024
#define HEIGHT 768


void interrupt (int s)
{
	exit(1);
}


void catchMe (const sensor_msgs::Image::ConstPtr &imagemsg)
{
	printf ("Hello World Imaging\n");
}


int main (int argc, char **argv)
{
	Rate loop (25);
	SceneManager &scene = SceneManager::getInstance(WIDTH, HEIGHT);
	signal (SIGINT, interrupt);

	// ROS Business
	ros::init (argc, argv, "video_view", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;

	CameraImageBackground::Ptr videoRect (new CameraImageBackground (rosnode, "/camera/image_raw", WIDTH, HEIGHT));
	scene.addObject (videoRect);

	scene.getCamera()->lookAt (
		Point3(0.5, -1, 0.5),
		Point3(0.5, 1, 0.5),
		Vector3(0, 0, 1));
	scene.getCamera()->perspective (45.0,
		(float)WIDTH / (float)HEIGHT,
		1.0, 100);

	while (true) {
		ros::spinOnce();
		scene.update();
		loop.sleep();
	}

	return 0;
}
