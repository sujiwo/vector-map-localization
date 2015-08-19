/*
 * CameraImageBackground.cpp
 *
 *  Created on: May 5, 2015
 *      Author: jiwo
 */

#include "CameraImageBackground.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>


CameraImageBackground::CameraImageBackground (
		ros::NodeHandle &n,
		const string &imt,
		int _width, int _height) :
	node (n),
	imageTopic (imt),
	imgtrans (node),
	disabled (false)
{
	rectbuf = NULL;
	width = _width, height = _height;
	glGenTextures (1, &texId);
	glGenBuffers (1, &pbo);
}


CameraImageBackground::~CameraImageBackground()
{
	//BitmapBackground::~BitmapBackground ();
}


void CameraImageBackground::initialize()
{
	camSub = imgtrans.subscribe (imageTopic, 10, &CameraImageBackground::imageCatch, this);
}


void CameraImageBackground::imageCatch (const sensor_msgs::Image::ConstPtr &imagemsg)
{
	if (rectbuf==NULL) {
		// initialize image buffer & framebuffer
		_bitmap = new uint8_t[BitmapSize];
		rectbuf = new Framebuffer (width, height);
	}

	// raw decoding
	CvImagePtr convImage = cv_bridge::toCvCopy(imagemsg, "rgb8");

	// downscaling
	cv::Mat bitmap (height, width, CV_8UC3, _bitmap);
	cv::resize (convImage->image, bitmap, bitmap.size(), 0, 0);

	// Row-reverse and flipping (rotating by 180 deg) are required for display in OpenGL
	cv::flip (bitmap, bitmap, 0);
	cv::flip (bitmap, bitmap, -1);
}


void CameraImageBackground::draw()
{
	if (disabled==true)
		return;
	if (rectbuf==NULL)
		return;
	else BitmapBackground::draw();
}


void CameraImageBackground::doDraw ()
{
}
