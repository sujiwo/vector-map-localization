/*
 * CameraImageBackground.h
 *
 *  Created on: May 5, 2015
 *      Author: jiwo
 */

#ifndef PROJECTOR_CAMERAIMAGEBACKGROUND_H_
#define PROJECTOR_CAMERAIMAGEBACKGROUND_H_

#include "BitmapBackground.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>


using std::string;
using cv_bridge::CvImagePtr;


class CameraImageBackground: public BitmapBackground
{
public:
	CameraImageBackground (ros::NodeHandle &n, const string &topic, int _width, int _height);
	virtual ~CameraImageBackground();

	void initialize ();

	typedef shared_ptr<CameraImageBackground> Ptr;

	void imageCatch (const sensor_msgs::Image::ConstPtr &imagemsg);

	void draw ();
	void doDraw ();
	void disable() { disabled = true; }


private:
	ros::NodeHandle &node;
	const string imageTopic;
	image_transport::ImageTransport imgtrans;
	image_transport::Subscriber camSub;
	//CvImagePtr convImage;
	bool disabled;
};

#endif /* PROJECTOR_CAMERAIMAGEBACKGROUND_H_ */
