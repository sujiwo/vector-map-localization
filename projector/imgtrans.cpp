#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <cstdlib>


using std::string;
using cv_bridge::CvImagePtr;

int defWidth = 1024,
	defHeight = 768;

image_transport::Publisher camPub;


void imageCatch (const sensor_msgs::Image::ConstPtr &imagemsg)
{
	static int frameId = 0;

	CvImagePtr convImage = cv_bridge::toCvCopy(imagemsg, "rgb8");
	cv::Mat bitmap (defHeight, defWidth, CV_8UC3);
	cv::resize (convImage->image, bitmap, bitmap.size(), 0, 0);

	//cv::flip (bitmap, bitmap, 0);
	//flipping
	cv::flip (bitmap, bitmap, -1);

	// change brightness
	bitmap = bitmap + cv::Scalar(50, 50, 50);

	std_msgs::Header header;
	header.seq = frameId++;
	header.frame_id = "CameraX";
	header.stamp = ros::Time::now();
	cv_bridge::CvImage bmpexport (header, "rgb8", bitmap);
	camPub.publish (bmpexport.toImageMsg());
}


int main (int argc, char *argv[])
{
	// ROS business
	ros::init(argc, argv, "map_points", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;

	image_transport::ImageTransport imgtrans (rosnode);
	image_transport::Subscriber camSub = imgtrans.subscribe(argv[1], 10, imageCatch);
	camPub = imgtrans.advertise ("processImage", 1);

	ros::spin();
}
