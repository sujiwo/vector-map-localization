/*
 * main.cpp
 *
 *  Created on: Jul 18, 2015
 *      Author: sujiwo
 */


#include <Eigen/Eigen>
#include <string>
#include <ros/ros.h>
#include "debug.h"
#include "Math.h"
#include "Rate.h"
#include "tf_eigen.h"
#include <signal.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#ifdef PUBLISH_GL
#include "sensor_msgs/fill_image.h"
#include "sensor_msgs/image_encodings.h"
#endif

#include <QtGui/QApplication>
#include "MatchWindow.h"
#include "RenderWidget.h"


using std::string;


const string cameraTopic ("camera1");
bool _doExit = false;
const int defaultRate = 50;
MatchWindow *mainwindow;


Eigen::Quaternionf orientation;
Point3 position;

#ifdef PUBLISH_GL
image_transport::ImageTransport *imgtrans;
image_transport::Publisher imgSend;
#endif


void interrupt (int s)
{
	_doExit = true;
}


void getTransform (Eigen::Quaternionf &ori, Point3 &pos)
{
	static tf::TransformListener listener;
	tf::StampedTransform transform;

	// target_frame    source_frame
	ros::Time t = ros::Time();
	listener.waitForTransform (cameraTopic, "japan_7", t, ros::Duration(10.0));
	listener.lookupTransform (cameraTopic, "japan_7", t, transform);

	tf2eigen (transform, ori, pos);
}


void image_data_recv (const sensor_msgs::Image::ConstPtr &imagemsg)
{
	// XXX: make getTransform new thread
	try {
		getTransform (orientation, position);
		mainwindow->setPose (position, orientation);
		mainwindow->update();

#ifdef PUBLISH_GL
		cv::Mat imageSynth;
		mainwindow->canvas()->getImage(imageSynth);
		sensor_msgs::Image msg;
		msg.header.stamp = ros::Time::now();
		sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::RGBA8, imageSynth.rows, imageSynth.cols, imageSynth.cols*4, imageSynth.data);
		imgSend.publish(msg);
#endif

	} catch (tf::TransformException &exc) {
		debug ("getTransform() failed");
	}

	cv_bridge::CvImageConstPtr convImage = cv_bridge::toCvShare(imagemsg, "rgb8");
	//cv::cvtColor(convImage->image, convImage->image, CV_BGR2RGB);
	mainwindow->setImageSource (convImage->image);
}


int main (int argc, char **argv)
{
	// ROS business
	ros::init(argc, argv, "vmslam", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;
	ros::Subscriber imgRecv = rosnode.subscribe (cameraTopic+"/image_raw", 10, image_data_recv);

#ifdef PUBLISH_GL
	imgtrans = new image_transport::ImageTransport (rosnode);
	imgSend = imgtrans->advertise ("/glimage", 1);
#endif

	// QT
	QApplication app (argc, argv);
	mainwindow = new MatchWindow ();
	mainwindow->show();

	Rate loop (defaultRate);
	signal (SIGINT, interrupt);

	while (true) {

		if (mainwindow->isClosed() == true || _doExit==true)
			break;

		ros::spinOnce();
		app.processEvents();
		loop.sleep();
	}

	delete (mainwindow);

#ifdef PUBLISH_GL
	delete (imgtrans);
#endif

	return 0;
}
