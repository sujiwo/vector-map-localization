/*
 * VectorMapMatchWindow.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: sujiwo
 */

#include <string>
#include <opencv2/opencv.hpp>
#include "MatchWindow.h"
#include "RenderWidget.h"
#include "debug.h"
#include "ImageMatch.h"
#include "tf_eigen.h"
#include "ImageDisplay.h"
#include <QtGui/QPushButton>
#include <QtGui/QComboBox>
#include <QtGui/QPainter>
#include <QtGui/QCheckBox>
#include "Pose.h"
#include "vector_map.h"
#include "PointSolver.h"


using std::string;


void insertObjects (RenderWidget *w, VectorMap *mapsrc);


// XXX: get real camera info
const struct {
float
	fx = 1150.96938467,
	fy = 1150.96938467,
	wi = 1920,
	hi = 1440,
	cx = 988.511326762,
	cy = 692.803953253;
} CameraInfo;

#define VECTOR_MAP_PATH "/var/tmp/nuvm"


MatchWindow::MatchWindow () :
	closed (false)
{
	centralWidget = new QWidget (this);
	this->setCentralWidget(centralWidget);

	mainLayout = new QVBoxLayout (centralWidget);
	centralWidget->setLayout (mainLayout);

	imagesContainer = new QWidget (centralWidget);
	imagesContainer->setLayout (new QHBoxLayout());
	mainLayout->addWidget (imagesContainer);

	glcanvas = new RenderWidget (MatchWindow::defaultImageWidth, MatchWindow::defaultImageHeight, centralWidget);

	imageDisplay = new ImageDisplay (
		QSize(MatchWindow::defaultImageWidth, MatchWindow::defaultImageHeight),
		glcanvas,
		centralWidget);
	imagesContainer->layout()->addWidget (imageDisplay);

	vmap = new VectorMap ();
	vmap->loadAll (VECTOR_MAP_PATH);

	// glcanvas must be added in widget hierarchy
	imagesContainer->layout()->addWidget (glcanvas);
	insertObjects (glcanvas, vmap);

	/* Poor man's toolbar */
	QWidget *btnBar = new QWidget (centralWidget);
	btnBar->setLayout(new QHBoxLayout());
	centralWidget->layout()->addWidget(btnBar);

	QPushButton *captureBtn = new QPushButton ("Capture", centralWidget);
	captureBtn->setFixedWidth(120);
	btnBar->layout()->addWidget(captureBtn);
	QObject::connect (captureBtn, SIGNAL(clicked()), this, SLOT(captureClicked()));

	QPushButton *searchBtn = new QPushButton ("Search", centralWidget);
	searchBtn->setFixedWidth(120);
	btnBar->layout()->addWidget(searchBtn);
	QObject::connect (searchBtn, SIGNAL(clicked()), this, SLOT(searchButtonClicked()));

	QPushButton *resetBtn = new QPushButton ("Reset", centralWidget);
	resetBtn->setFixedWidth(120);
	btnBar->layout()->addWidget(resetBtn);
	QObject::connect (resetBtn, SIGNAL(clicked()), this, SLOT(resetButtonClicked()));

	QComboBox *imageTypeSelector = new QComboBox (centralWidget);
	imageTypeSelector->setFixedWidth(210);
	imageTypeSelector->insertItem(ImageDisplay::ImageTypeRgb, "RGB");
	imageTypeSelector->insertItem(ImageDisplay::ImageTypeGray, "Grayscale");
	imageTypeSelector->insertItem(ImageDisplay::ImageTypeGrayProcessed, "Processed Grayscale");
	QObject::connect (imageTypeSelector, SIGNAL(currentIndexChanged(int)), imageDisplay, SLOT(changeImageType(int)));
	btnBar->layout()->addWidget(imageTypeSelector);

	QCheckBox *toggleMap = new QCheckBox ("Show Map", centralWidget);
	QObject::connect (toggleMap, SIGNAL(toggled(bool)), imageDisplay, SLOT(toggleMapProjection(bool)));
	toggleMap->setChecked(true);
	btnBar->layout()->addWidget (toggleMap);

	QWidget *poseBox = new QWidget (centralWidget);
	poseBox->setLayout(new QHBoxLayout());
	mainLayout->addWidget(poseBox);

	QWidget *posContainer = new QWidget (centralWidget);
	posContainer->setLayout(new QVBoxLayout());
	posContainer->layout()->addWidget(new QLabel("Position"));
	posContainer->layout()->addWidget(wposx = new LabelWithTextBox("X"));
	posContainer->layout()->addWidget(wposy = new LabelWithTextBox("Y"));
	posContainer->layout()->addWidget(wposz = new LabelWithTextBox("Z"));
	QObject::connect (wposx, SIGNAL(doPoseChange(QString,int)),
		this, SLOT(PoseChangeManual(QString,int)));
	QObject::connect (wposy, SIGNAL(doPoseChange(QString,int)),
		this, SLOT(PoseChangeManual(QString,int)));
	QObject::connect (wposz, SIGNAL(doPoseChange(QString,int)),
		this, SLOT(PoseChangeManual(QString,int)));

	QWidget *oriContainer = new QWidget (centralWidget);
	oriContainer->setLayout (new QVBoxLayout());
	oriContainer->layout()->addWidget(new QLabel("Orientation"));
	oriContainer->layout()->addWidget(wroll = new LabelWithTextBox("Bank"));
	oriContainer->layout()->addWidget(wpitch = new LabelWithTextBox("Elevation"));
	oriContainer->layout()->addWidget(wyaw = new LabelWithTextBox("Heading"));
	QObject::connect (wroll, SIGNAL(doPoseChange(QString,int)),
		this, SLOT(PoseChangeManual(QString,int)));
	QObject::connect (wpitch, SIGNAL(doPoseChange(QString,int)),
		this, SLOT(PoseChangeManual(QString,int)));
	QObject::connect (wyaw, SIGNAL(doPoseChange(QString,int)),
		this, SLOT(PoseChangeManual(QString,int)));

	poseBox->layout()->addWidget(posContainer);
	poseBox->layout()->addWidget(oriContainer);

	cameraFix = new PointSolver (vmap, glcanvas, MatchWindow::defaultImageWidth, MatchWindow::defaultImageHeight);

	return;
}


MatchWindow::~MatchWindow()
{
	delete (vmap);
}


void MatchWindow::updatePoseText ()
{
	Point3 worldPos = glcanvas->pose()->getWorldPosition();
	double
		heading = (glcanvas->pose()->getHeading()) * 180.0/M_PI,
		elevation = glcanvas->pose()->getElevation() * 180.0/M_PI,
		bank = glcanvas->pose()->getBank() * 180.0/M_PI;
	wposx->setText (worldPos.x());
	wposy->setText (worldPos.y());
	wposz->setText (worldPos.z());
	wyaw->setText (heading);
	wpitch->setText (elevation);
	wroll->setText (bank);
}


void copyCvToQImage (const cv::Mat &image, QImage *target)
{

	for (int i=0; i<image.rows; i++) {
		memcpy (target->scanLine(i), image.ptr(i), image.step);
	}
}


void overlay (QImage *target, QImage *source)
{
	QPainter painter (target);
	QPointF o(0, 0);
	painter.drawImage (o, *source);
}


void MatchWindow::setImageSource (const cv::Mat &image)
{
	imageDisplay->updateImage(image);
	imageDisplay->repaint();
}


void MatchWindow::closeEvent (QCloseEvent *evt)
{
	closed = true;
	QMainWindow::closeEvent(evt);
}


void MatchWindow::setPose (const Point3 &position, const Quaternion &orientation)
{
	glcanvas->pose()->set (position, orientation);

	updatePoseText ();
}


void MatchWindow::update()
{
	glcanvas->pose()->reset();
	glcanvas->draw();
}


#include <cstdio>


void MatchWindow::captureClicked()
{
	imageDisplay->dump();
}


void MatchWindow::searchButtonClicked ()
{
//	cv::Mat *mask = &imageDisplay->getMask();
//	Point3 pt = imageSearch (imageDisplay->getProcessedGray(), glcanvas, mask);
//	std::cout << pt << std::endl;
//	glcanvas->pose()->offset(pt);

	cv::Mat pcGray = imageDisplay->getProcessedGray();
	cameraFix->solve (&pcGray, glcanvas->pose()->getPosition(), glcanvas->pose()->getOrientation());

	glcanvas->draw(true);
	imageDisplay->repaint();

	updatePoseText ();
}


void MatchWindow::resetButtonClicked()
{
	glcanvas->pose()->reset();
	glcanvas->draw (true);
	imageDisplay->repaint();

	updatePoseText ();
}


void MatchWindow::PoseChangeManual(QString btnId, int button)
{
	Point3 &cpos = glcanvas->pose()->getPositionOffset();
	double roll = 0, pitch = 0, yaw = 0;

	if (btnId=="X") {
		if (button==PoseTuneButton::Plus)
			cpos.x() += dX;
		else if (button==PoseTuneButton::Minus)
			cpos.x() -= dX;
	}
	else if (btnId=="Y") {
		if (button==PoseTuneButton::Plus)
			cpos.y() += dY;
		else if (button==PoseTuneButton::Minus)
			cpos.y() -= dY;
	}
	else if (btnId=="Z") {
		if (button==PoseTuneButton::Plus)
			cpos.z() += dZ;
		else if (button==PoseTuneButton::Minus)
			cpos.z() -= dZ;
	}
	else if (btnId=="Bank") {
		if (button==PoseTuneButton::Plus)
			glcanvas->pose()->offsetRoll (dR);
		else if (button==PoseTuneButton::Minus)
			glcanvas->pose()->offsetRoll (-dR);
	}
	else if (btnId=="Elevation") {
		if (button==PoseTuneButton::Plus)
			glcanvas->pose()->offsetPitch(dP);
		else if (button==PoseTuneButton::Minus)
			glcanvas->pose()->offsetPitch(-dP);
	}
	else if (btnId=="Heading") {
		if (button==PoseTuneButton::Plus)
			glcanvas->pose()->offsetYaw(dYw);
		else if (button==PoseTuneButton::Minus)
			glcanvas->pose()->offsetYaw(-dYw);
	}

	glcanvas->draw(true);
	imageDisplay->repaint();
	updatePoseText ();
}


LabelWithTextBox::LabelWithTextBox (const string &textlabel, QWidget *parent) :
	QWidget (parent),
	text (textlabel)
{
	this->setLayout(new QHBoxLayout());
	textEntry = new QLineEdit (this);
	this->layout()->addWidget(new QLabel(QString(textlabel.c_str())));
	this->layout()->addWidget(textEntry);
	textEntry->setReadOnly(true);

	layout()->addWidget(plus = new QPushButton ("+"));
	plus->setFixedWidth(50);
	QObject::connect (this->plus, SIGNAL(clicked()), this, SLOT(plusClick()));
	layout()->addWidget(minus = new QPushButton ("-"));
	minus->setFixedWidth(50);
	QObject::connect (this->minus, SIGNAL(clicked()), this, SLOT(minusClick()));
}


void LabelWithTextBox::plusClick()
{
	emit doPoseChange (QString::fromStdString(text), MatchWindow::PoseTuneButton::Plus);
}


void LabelWithTextBox::minusClick()
{
	emit doPoseChange (QString::fromStdString(text), MatchWindow::PoseTuneButton::Minus);
}


/* For testing purposes */
#include "DrawObjects.h"
//#include "LineList.h"
#include "LinesRaw.h"
#include "TrafficLights.h"
#include "PolesDraw.h"
#include "DirectionArrow.h"
#include "sensor_msgs/CameraInfo.h"
#include "AreaList.h"
#include "Gutters.h"

//void insertObjects (RenderWidget *w)
//{
//	Point3 p1 (0, 0, -2),
//			p2 (1, 0, -2),
//			p3 (1, 1, -2),
//			p4 (0, 1, -2);
//	PointList *rect = new PointList (w);
//	rect->add (p1);
//	rect->add (p2);
//	rect->add (p3);
//	rect->add (p4);
//	rect->setPointSize(4.0);
//	rect->setColor (Vector3(1.0, 1.0, 0.0));
//	w->addObject(rect);
//
//	w->getCamera()->lookAt (
//		Point3 (0, 0, 2),
//		Point3 (1, 1, -2),
//		Vector3 (0, 1, 0)
//	);
//	w->getCamera()->perspective(45.0, (float)MatchWindow::defaultImageWidth/(float)MatchWindow::defaultImageHeight, 1.5, 6);
//}


void insertObjects (RenderWidget *w, VectorMap *mapsrc)
{
	// XXX: May need to remove this part
//	MapPoints *map = new MapPoints (mapsrc, w);
//	map->setPointSize (3.0);
//	map->setColor (255, 255, 0);
//	w->addObject (map);
//	map->filterDistance = false;

	// XXX: need to tune scale
//	const double scale = 0.05;
//	LineList *lineList = new LineList (mapsrc, scale, w);
//	w->addObject(lineList);
	LinesRaw *lnsz = new LinesRaw (mapsrc, w);
	w->addObject(lnsz);

	TrafficLights *trafficLights = new TrafficLights (mapsrc, w);
	w->addObject(trafficLights);

	PolesDraw *poles = new PolesDraw (mapsrc, w);
	w->addObject(poles);

//	DirectionArrow *arrow = new DirectionArrow (w);
//	w->addObject(arrow);
//	Gutters *gutters = new Gutters (mapsrc, w);
//	w->addObject(gutters);
//	AreaList *area = new AreaList (mapsrc, w);
//	w->addObject(area);

	// XXX: get real camera info
	//w->getCamera()->perspective(45.0, (float)MatchWindow::defaultImageWidth/(float)MatchWindow::defaultImageHeight, 1.5, 10000);
	w->getCamera()->projectionMatrixFromCameraInfo(CameraInfo.fx, CameraInfo.fy, CameraInfo.wi, CameraInfo.hi, CameraInfo.cx, CameraInfo.cy);

	//delete (map);
}
