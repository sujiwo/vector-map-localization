/*
 * ImageDisplay.cpp
 *
 *  Created on: Aug 7, 2015
 *      Author: sujiwo
 */

#include "ImageDisplay.h"
#include "RenderWidget.h"
#include "debug.h"
#include "ImageMatch.h"


#define AddPixel 0
const std::string maskFile ("/var/tmp/mask.png");


QImage qGrayScale (int width, int height)
{
	QImage grImage(width, height, QImage::Format_Indexed8);
	QVector<QRgb> clrTable;
	for (int i=0; i<256; i++)
		clrTable.push_back (QColor(i, i, i).rgb());
	grImage.setColorTable (clrTable);

	return grImage;
}


QImage qGrayScaleFromRgba (QImage &source)
{
	QImage gray = qGrayScale (source.width(), source.height());
	for (int ii = 0; ii < source.height(); ii++) {
	    uchar* scan = source.scanLine(ii);
	    int depth = 4;
	    for (int jj = 0; jj < source.width(); jj++) {
	    	QRgb* rgbpixel = reinterpret_cast<QRgb*>(scan + jj*depth);
	        int grayVal = qGray(*rgbpixel);
	        gray.setPixel(jj, ii, grayVal);
	    }
	}
	return gray;
}


QImage qGrayScaleFromRgb (QImage &source)
{
	QImage gray = qGrayScale (source.width(), source.height());
	for (int ii = 0; ii < source.height(); ii++) {
	    uchar* scan = source.scanLine(ii);
	    int depth = 3;
	    for (int jj = 0; jj < source.width(); jj++) {
	        unsigned char* rgb = (scan + jj*depth);
	        int grayVal = qGray(rgb[0], rgb[1], rgb[2]);
	        gray.setPixel(jj, ii, grayVal);
	    }
	}
	return gray;
}


QImage qGrayScale (QImage &source)
{
	if (source.format() == QImage::Format_ARGB32 || source.format()==QImage::Format_ARGB32_Premultiplied)
		return qGrayScaleFromRgba (source);
	else if (source.format() == QImage::Format_RGB888)
		return qGrayScaleFromRgb (source);
}


ImageDisplay::ImageDisplay(QSize imgsize, RenderWidget *canvas, QWidget *parent) :
	mySize (imgsize),
	glcanvas (canvas),

	cameraImage (cv::Mat (imgsize.height(), imgsize.width(), CV_8UC3)),
	grayImage (cv::Mat (imgsize.height(), imgsize.width(), CV_8UC1)),
	processedGrayImage (cv::Mat (imgsize.height(), imgsize.width(), CV_8UC1)),

	// XXX: please prepare the mask file
	mask (cv::imread(maskFile, cv::IMREAD_GRAYSCALE)),

	displayType (ImageTypeRgb),

	QWidget (parent),

	mapShown (true)
{
	setFixedSize (imgsize.width()+AddPixel, imgsize.height()+AddPixel);

	qCameraImage = new QImage (cameraImage.data, imgsize.width(), imgsize.height(), QImage::Format_RGB888);
	qGrayImage = new QImage (grayImage.data, imgsize.width(), imgsize.height(), QImage::Format_Indexed8);
	qProcessedGrayImage = new QImage (processedGrayImage.data, imgsize.width(), imgsize.height(), QImage::Format_Indexed8);

	cv::resize (mask, mask, cameraImage.size(), 0, 0);
	qMask = new QImage (mask.data, imgsize.width(), imgsize.height(), QImage::Format_Indexed8);

	QVector<QRgb> clrTable;
	for (int i=0; i<256; i++)
		clrTable.push_back (QColor(i, i, i).rgb());
	qGrayImage->setColorTable (clrTable);
	qProcessedGrayImage->setColorTable (clrTable);
	qMask->setColorTable (clrTable);

//	glcanvas->setStencil (&mask);
}


ImageDisplay::~ImageDisplay()
{
	delete (qCameraImage);
	delete (qGrayImage);
	delete (qProcessedGrayImage);
}


void ImageDisplay::changeImageType(int type)
{
	displayType = type;
	this->repaint();
}


void ImageDisplay::toggleMapProjection (bool shown)
{
	mapShown = shown;
	this->repaint ();
}


void ImageDisplay::updateImage (const cv::Mat &src)
{
	if (src.type() != cameraImage.type())
		throw "Mismatch image type";

	cv::resize (src, cameraImage, cameraImage.size(), 0, 0);
	cv::cvtColor(cameraImage, grayImage, CV_RGB2GRAY);
	cv::bitwise_and (grayImage, mask, grayImage);

	imagePipeline (grayImage, processedGrayImage);
}


void ImageDisplay::applyOverlay ()
{
	QPainter wgpaint (this);
	QImage *iptr;

	if (displayType==ImageTypeRgb)
		iptr = qCameraImage;
	else if (displayType==ImageTypeGray)
		iptr = qGrayImage;
	else if (displayType==ImageTypeGrayProcessed)
		iptr = qProcessedGrayImage;

//	QPainter wgmask (this->glcanvas->getFramebuffer());
//	wgmask.drawImage (QPoint(0, 0), *qMask);

	wgpaint.drawImage (QPoint(0, 0), *iptr);
	if (mapShown==true) {
		wgpaint.drawImage (QPoint(0, 0), glcanvas->getFramebuffer()->toImage());
	}
}




void ImageDisplay::dump()
{
	cv::imwrite ("/tmp/real.png", cameraImage);
	cv::imwrite ("/tmp/gray.png", grayImage);
	cv::imwrite ("/tmp/processed.png", processedGrayImage);
	QImage synthImg = glcanvas->getFramebuffer()->toImage();
	synthImg.save("/tmp/gl.png");

//	QImage synthGrayImg = qGrayScale (synthImg);
//	synthGrayImg.save ("/tmp/glgray.png");
	cv::Mat synthImg2 = RenderWidget::convert2Cv(&synthImg);
	cv::Mat synthGray (synthImg.height(), synthImg.width(), CV_8UC1);
	cv::cvtColor(synthImg2, synthGray, CV_RGBA2GRAY);
	cv::threshold(synthGray, synthGray, 10, 255, cv::THRESH_BINARY);
	cv::bitwise_and (synthGray, mask, synthGray);
	cv::imwrite ("/tmp/glgray.png", synthGray);

	// Write pose parameters
	FILE *outparam = fopen ("/tmp/params.txt", "w");
	Point3 &pos = glcanvas->pose()->getPosition();
	Quaternion &ori = glcanvas->pose()->getOrientation();

	fprintf (outparam, "Position: %f %f %f\n", pos.x(), pos.y(), pos.z());
	fprintf (outparam, "Orientation: %f %f %f %f\n", ori.w(), ori.x(), ori.y(), ori.z());
//	fprintf (outparam, "Camera: %f %f %f %f %f %f\n", CameraInfo.fx, CameraInfo.fy, CameraInfo.wi, CameraInfo.hi, CameraInfo.cx, CameraInfo.cy);
	fclose (outparam);

	debug ("Captured");
}


void ImageDisplay::paintEvent (QPaintEvent *evt)
{
	applyOverlay ();
}
