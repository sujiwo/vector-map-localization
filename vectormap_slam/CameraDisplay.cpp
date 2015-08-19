/*
 * CameraDisplay.cpp
 *
 *  Created on: Jul 27, 2015
 *      Author: sujiwo
 */

#include "CameraDisplay.h"
#include "RenderWidget.h"
#include "debug.h"



CameraDisplay::CameraDisplay (RenderWidget *w, int width, int height, QWidget *parent) :
	glcanvas (w),
	rect (width, height)
{
	QGLWidget (parent, glcanvas);
	if (this->isSharing())
		debug ("Not sharing context");
	setFixedSize (width, height);
}


void CameraDisplay::initializeGL()
{
	if (glcanvas->context() != this->context())
		debug ("Context not same");
}


void CameraDisplay::update(const cv::Mat &newimg)
{
	source = newimg;
	this->draw ();
}


void CameraDisplay::paintGL()
{
	this->draw ();
}


void CameraDisplay::draw ()
{
	if (source.rows==0)
		return;

	cv::Mat fbclone = source.clone();
	cv::flip (fbclone, fbclone, -1);
	cv::flip (fbclone, fbclone, 1);

	makeCurrent ();
	glClear (GL_COLOR_BUFFER_BIT);
	glDrawPixels (rect.width(), rect.height(), GL_RGB, GL_UNSIGNED_BYTE, fbclone.data);
	// xxx: copy from RenderWidget
	QRect rsrc (0, 0, rect.width(), rect.height());
	QGLFramebufferObject::blitFramebuffer(0, rsrc, glcanvas->getFramebuffer(), rsrc);
	swapBuffers ();
	doneCurrent ();
}
