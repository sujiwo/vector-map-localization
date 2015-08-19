#include <iostream>
#include "debug.h"
#include <QtGui/QApplication>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include "Rate.h"
#include "VectorMapMatchWindow.h"
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>



VectorMapMatchWindow *mainwindow;


void gnss_data_recv (const geometry_msgs::PoseStamped::ConstPtr pose)
{
//	double
//		posx = pose->pose.position.x,
//		posy = pose->pose.position.y,
//		posz = pose->pose.position.z;
//	mainwindow->setPositionText(posx, posy, posz);
	mainwindow->setPose (pose);
}


void image_data_recv (const sensor_msgs::Image::ConstPtr &imagemsg)
{
	cv_bridge::CvImagePtr convImage = cv_bridge::toCvCopy(imagemsg, "rgb8");
	//cv::cvtColor(convImage->image, convImage->image, CV_BGR2RGB);
	mainwindow->setImageSource (convImage->image);
}



int main (int argc, char **argv)
{
	if (argc != 3) {
		std::cerr << "Usage: " << argv[0] << " <vector_map_path> <camera_info.yaml>" << std::endl;
		exit (1);
	}

	QApplication app (argc, argv);

	ros::init(argc, argv, "vmmatch", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;
	ros::Subscriber gnssRecv = rosnode.subscribe ("gnss_pose", 100, gnss_data_recv);
	//image_transport::ImageTransport imageTrans (rosnode);
	debug ("GL Started");
	// Baris berikut ini membuat masalah dengan glsldb
	//image_transport::Subscriber imageSubs = imageTrans.subscribe ("/image_raw", 10, image_data_recv);
	ros::Subscriber imgRecv = rosnode.subscribe ("/image_raw", 10, image_data_recv);
	debug ("Image Subscribed");

	mainwindow = new VectorMapMatchWindow ();
	mainwindow->show();

	Rate rate (100);
	while (true) {
		if (mainwindow->isClosed() == true)
			break;
		ros::spinOnce();
		app.processEvents();
		mainwindow->render();
		rate.sleep();
	}

	delete (mainwindow);
	return 0;
}
