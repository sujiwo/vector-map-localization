/*
 * VectorMapMatchWindow.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: sujiwo
 */

#include <string>
#include "VectorMapMatchWindow.h"
#include "VectorMapObjects.h"
#include <camera_calibration_parsers/parse.h>



using std::string;


VectorMapMatchWindow::VectorMapMatchWindow() :
	imageSrcIsSet (false),
	closed (false),
	imagebmp (NULL),
	glcanvas (NULL)
{
	centralWidget = new QWidget (this);
	this->setCentralWidget(centralWidget);

	mainLayout = new QVBoxLayout (centralWidget);
	centralWidget->setLayout (mainLayout);

	imagesContainer = new QWidget (centralWidget);
	imagesContainer->setLayout (new QHBoxLayout());
	mainLayout->addWidget (imagesContainer);

	imageSrc = new QLabel ();
	imagesContainer->layout()->addWidget (imageSrc);

	glcanvas = new VectorMapRenderWidget (centralWidget);
	imagesContainer->layout()->addWidget (glcanvas);

	QWidget *poseBox = new QWidget (centralWidget);
	poseBox->setLayout(new QHBoxLayout());
	mainLayout->addWidget(poseBox);

	QWidget *posContainer = new QWidget (centralWidget);
	posContainer->setLayout(new QVBoxLayout());
	posContainer->layout()->addWidget(new QLabel("Position"));
	wposx = new QLineEdit ();
	wposx->setReadOnly(true);
	posContainer->layout()->addWidget(wposx);
	wposy = new QLineEdit ();
	wposy->setReadOnly(true);
	posContainer->layout()->addWidget(wposy);
	wposz = new QLineEdit ();
	wposz->setReadOnly(true);
	posContainer->layout()->addWidget(wposz);

	QWidget *oriContainer = new QWidget (centralWidget);
	oriContainer->setLayout (new QVBoxLayout());
	oriContainer->layout()->addWidget(new QLabel("Orientation"));
	oriContainer->layout()->addWidget(new LabelWithTextBox("Roll"));
	oriContainer->layout()->addWidget(new LabelWithTextBox("Pitch"));
	oriContainer->layout()->addWidget(new LabelWithTextBox("Yaw"));

	poseBox->layout()->addWidget(posContainer);
	poseBox->layout()->addWidget(oriContainer);

	// Map objects
	Point3 p1 (0, 0, 0),
			p2 (1, 0, 0),
			p3 (1, 1, 0),
			p4 (0, 1, 0);
	TRectangle *rect = new TRectangle (p1, p2, p3, p4, glcanvas);
	glcanvas->addObject(rect);

	// Load vector map
	string vmpath = QCoreApplication::instance()->arguments().at(1).toStdString();
	MapPoints *vmmap = new MapPoints (vmpath, glcanvas);
	vmmap->setColor (Vector3 (1.0, 1.0, 1.0));
	vmmap->setPointSize(2.0);
	glcanvas->addObject(vmmap);

	// Camera Info
	string cameraInfoFile = QCoreApplication::instance()->arguments().at(2).toStdString();
	string cameraName;
	camera_calibration_parsers::readCalibration(cameraInfoFile, cameraName, cameraInfo);

	return;
}


VectorMapMatchWindow::~VectorMapMatchWindow()
{}


void VectorMapMatchWindow::setPositionText (const double &x, const double &y, const double &z)
{
	QString sposx = QString::number(x);
	wposx->setText(sposx);
	QString sposy = QString::number(y);
	wposy->setText(sposy);
	QString sposz = QString::number(z);
	wposz->setText(sposz);
}


void VectorMapMatchWindow::setImageSource (const cv::Mat &image)
{
	if (imageSrcIsSet == false) {
		imagebmp = new QImage (image.data, image.cols, image.rows, QImage::Format_RGB888);
		imageSrcIsSet = true;
		glcanvas->setSize (image.cols, image.rows);
	}
	else {
		delete (imagebmp);
		imagebmp = new QImage (image.data, image.cols, image.rows, QImage::Format_RGB888);
	}

	imageSrc->setPixmap(QPixmap::fromImage(*imagebmp));
}


void VectorMapMatchWindow::closeEvent (QCloseEvent *evt)
{
	closed = true;
	QMainWindow::closeEvent(evt);
}


void VectorMapMatchWindow::setPose (const geometry_msgs::PoseStamped::ConstPtr pose)
{
	double position[3] = {
		pose->pose.position.x,
		pose->pose.position.y,
		pose->pose.position.z
	};
	double orientation[4] = {
		pose->pose.orientation.x,
		pose->pose.orientation.y,
		pose->pose.orientation.z,
		pose->pose.orientation.w
	};
	setPositionText(position[0], position[1], position[2]);

	//glcanvas->setPose(position, orientation);
}


LabelWithTextBox::LabelWithTextBox (const string &textlabel, QWidget *parent) :
	QWidget (parent)
{
	this->setLayout(new QHBoxLayout());
	textEntry = new QLineEdit (this);
	this->layout()->addWidget(new QLabel(QString(textlabel.c_str())));
	this->layout()->addWidget(textEntry);
	textEntry->setReadOnly(true);
}
