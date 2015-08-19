/*
 * CameraDisplay.h
 *
 *  Created on: Jul 27, 2015
 *      Author: sujiwo
 */

#ifndef CAMERADISPLAY_H_
#define CAMERADISPLAY_H_

#include <QtOpenGL/QtOpenGL>
#include <opencv2/opencv.hpp>


class RenderWidget;


class CameraDisplay: public QGLWidget
{
Q_OBJECT

public:
	CameraDisplay (RenderWidget *w, int width, int height, QWidget *parent=NULL);

	void initializeGL ();
	void paintGL ();
	void update (const cv::Mat &sourceImage);

private:
	RenderWidget *glcanvas;
	void draw ();

	cv::Mat source;
};


#endif /* CAMERADISPLAY_H_ */
