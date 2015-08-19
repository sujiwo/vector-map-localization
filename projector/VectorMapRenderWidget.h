/*
 * VectorMapRenderWidget.h
 *
 *  Created on: Jul 7, 2015
 *      Author: sujiwo
 */

#ifndef VECTORMAPRENDERWIDGET_H_
#define VECTORMAPRENDERWIDGET_H_


#include <QtOpenGL/QtOpenGL>
#include <QGLViewer/qglviewer.h>
#include <vector>
#include "Camera.h"
#include "VectorMapObjects.h"


using std::vector;


class VectorMapRenderWidget: public QGLViewer {
Q_OBJECT
public:
	VectorMapRenderWidget(QWidget *parent);
	virtual ~VectorMapRenderWidget();

	void setSize (int width, int height);

	Camera* getCamera() { return camera; }
	QGLShaderProgram* getShader() { return shader; }
	void update ();

	inline void forceInit (int w, int h)
	{
		initializeGL ();
		setSize (w, h);
	}

	inline void addObject (VectorMapObject* d)
	{ drawObjects.push_back(d); }

	void setPose (double position[3], double orientation[4]);


protected:
	void initializeGL ();
	void paintGL ();
	void resizeGL (int w, int h);

private:
	Camera *camera;
	QGLShaderProgram *shader;

	vector<VectorMapObject*> drawObjects;
	void cameraToShader ();

	bool hasInit;
};


class TGLWidget : public QGLWidget {
Q_OBJECT
public:
	TGLWidget (QWidget *parent);
	~TGLWidget ();

protected:
	void initializeGL ();
	void paintGL ();
	void resizeGL ();
};


#endif /* VECTORMAPRENDERWIDGET_H_ */
