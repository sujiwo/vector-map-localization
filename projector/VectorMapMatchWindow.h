/*
 * VectorMapMatchWindow.h
 *
 *  Created on: Jul 6, 2015
 *      Author: sujiwo
 */

#ifndef SOURCE_DIRECTORY__PROJECTOR_VECTORMAPMATCHWINDOW_H_
#define SOURCE_DIRECTORY__PROJECTOR_VECTORMAPMATCHWINDOW_H_


#include <QtGui/QMainWindow>
#include <QtGui/QWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLineEdit>
#include <QtGui/QLabel>
#include <QtGui/QImage>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include "VectorMapRenderWidget.h"
#include <geometry_msgs/PoseStamped.h>



class VectorMapMatchWindow: public QMainWindow {
	Q_OBJECT

public:
	VectorMapMatchWindow();
	virtual ~VectorMapMatchWindow();

	void setPositionText (const double &x, const double &y, const double &z);
	void setImageSource (const cv::Mat &image);

	bool isClosed ()
	{ return closed; }
	int getImageWidth ()
	{ return imagebmp->width(); }
	int getImageHeight ()
	{ return imagebmp->height(); }

	inline void render () { if (glcanvas!=NULL) glcanvas->update(); }

	void setPose (const geometry_msgs::PoseStamped::ConstPtr pose);


protected:
	QWidget *centralWidget;
	QVBoxLayout *mainLayout;

	QWidget *imagesContainer;

	QLabel *imageSrc;
	QImage *imagebmp;
	VectorMapRenderWidget *glcanvas;
	bool imageSrcIsSet;

	// position & orientation
	QLineEdit *wposx, *wposy, *wposz;
	QLineEdit *wroll, *wpitch, *wyaw;

	bool closed;
	void closeEvent (QCloseEvent *event);

	sensor_msgs::CameraInfo cameraInfo;
};


class LabelWithTextBox: public QWidget
{
Q_OBJECT
public:
	LabelWithTextBox (const string &textlabel, QWidget *parent=NULL);
	void setText (const string &text)
	{ textEntry->setText(QString::fromStdString(text)); }

protected:
	QLineEdit *textEntry;
};

#endif /* SOURCE_DIRECTORY__PROJECTOR_VECTORMAPMATCHWINDOW_H_ */
