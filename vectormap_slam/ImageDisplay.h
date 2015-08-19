/*
 * ImageDisplay.h
 *
 *  Created on: Aug 7, 2015
 *      Author: sujiwo
 */

#ifndef _IMAGEDISPLAY_H_
#define _IMAGEDISPLAY_H_

#include <QtGui/QWidget>
#include <QtGui/QImage>
#include <opencv2/core/core.hpp>



class RenderWidget;


class ImageDisplay: public QWidget
{
Q_OBJECT

public:
	ImageDisplay (QSize imgsize, RenderWidget *canvas, QWidget *parent=NULL);
	virtual ~ImageDisplay();

	cv::Mat& getImage () { return cameraImage; }
	cv::Mat& getGray () { return grayImage; }
	cv::Mat& getProcessedGray () { return processedGrayImage; }
	cv::Mat& getMask () { return mask; }

	void updateImage (const cv::Mat &src);

	void dump ();

	void paintEvent (QPaintEvent *event);

	enum {
		ImageTypeRgb,
		ImageTypeGray,
		ImageTypeGrayProcessed
	};

public slots:
	void changeImageType (int type);
	void toggleMapProjection (bool shown);


protected:
	QSize mySize;

	cv::Mat cameraImage;
	cv::Mat grayImage;
	cv::Mat processedGrayImage;
	cv::Mat mask;

	QImage *qCameraImage,
		*qGrayImage,
		*qProcessedGrayImage,
		*qMask;

	RenderWidget *glcanvas;
	QPixmap *compositeImage;

	int displayType;

	bool mapShown;

	void applyOverlay ();
};

#endif /* _IMAGEDISPLAY_H_ */
