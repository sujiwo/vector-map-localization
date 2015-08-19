/*
 * dbReadApp.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: sujiwo
 */

#include "dbReadApp.h"
#include <opencv2/core/core.hpp>
#include <QtGui/QImage>


dbReadApp::dbReadApp (ImageDB *dbsrc, int argc, char **argv) :
	QApplication (argc, argv),
	data (dbsrc)
{
	window = new QMainWindow ();
	centralWidget = new QWidget (window);

	mainLayout = new QVBoxLayout (centralWidget);
	centralWidget->setLayout (mainLayout);
	window->setCentralWidget (centralWidget);

	image1 = new QLabel (centralWidget);
	updateImage (1);
	mainLayout->addWidget (image1);

	window->show();
}


dbReadApp::~dbReadApp()
{
	// TODO Auto-generated destructor stub
}


void dbReadApp::updateImage (int id)
{

}


QPixmap dbReadApp::imageDataToPixmap (int id, Point3 &position, Quaternion &orientation)
{
	cv::Mat image;
	data->get (id, position, orientation, image);
	QImage *imgbuffer = new QImage (image.data, image.cols, image.rows, QImage::Format_RGB888);
	return QPixmap::fromImage (*imgbuffer);
}
