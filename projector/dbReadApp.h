/*
 * dbReadApp.h
 *
 *  Created on: Jun 24, 2015
 *      Author: sujiwo
 */

#ifndef SOURCE_DIRECTORY__PROJECTOR_DBREADAPP_H_
#define SOURCE_DIRECTORY__PROJECTOR_DBREADAPP_H_


#include <QtGui/QApplication>
#include <QtGui/QWidget>
#include <QtGui/QHBoxLayout>
#include <QtGui/QVBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPixmap>
#include <QtGui/QMainWindow>
#include "Math.h"
#include "ImageDB.h"



#define ImageWidth 640
#define ImageHeight 480



class dbReadApp : public QApplication
{
public:
	dbReadApp (ImageDB *dbsrc, int argc, char **argv);
	virtual ~dbReadApp();


protected:

	// window elements
	ImageDB *data;
	QMainWindow *window;
	QWidget *centralWidget;
	QVBoxLayout *mainLayout;

	// display elements
	QLabel *image1;

	QPixmap imageDataToPixmap (int id, Point3 &position, Quaternion &orientation);
	void updateImage (int id);
};

#endif /* SOURCE_DIRECTORY__PROJECTOR_DBREADAPP_H_ */
