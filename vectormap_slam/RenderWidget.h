/*
 * RenderWidget.h
 *
 *  Created on: Jul 7, 2015
 *      Author: sujiwo
 */

#ifndef RENDERWIDGET_H_
#define RENDERWIDGET_H_


#include <QtOpenGL/QtOpenGL>
#include <QtGui/QImage>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Camera.h"
#include "DrawObjects.h"
#include "Pose.h"


using std::vector;

/*
 * This widget should be invisible
 * To grab the image, copy the contents of framebuffer
 */


class RenderWidget: public QGLWidget {
Q_OBJECT
public:

	enum ImageFormat
	{
		RGB,
		RGBA,
		Gray
	};

	RenderWidget (int wWidth, int wHeight, QWidget *parent);
	virtual ~RenderWidget();

	Camera* getCamera() { return camera; }
	QGLShaderProgram* getShader() { return shader; }
	void update ();
	QGLFramebufferObject* getFramebuffer () { return renderBuffer; }

	inline void addObject (VectorMapObject* d)
	{ drawObjects.push_back(d); }

	void draw (bool useOffset=false);

	inline QImage getImage () { return renderBuffer->toImage();  }

	void getImage (cv::Mat &cvmat);

	void cameraToShader (QGLShaderProgram *shaderPrg);

	static cv::Mat convert2Cv (QImage *isrc);

	Pose* pose () { return &mypose; }

	void setStencil (cv::Mat *stencilImage);

protected:
	void initializeGL ();
	void paintGL ();

private:
	Camera *camera;
	QGLShaderProgram *shader;
	QGLFramebufferObject *renderBuffer;

	vector<VectorMapObject*> drawObjects;
	void cameraToShader ();
	void render ();

	QSize realsize;
	Pose mypose;

	cv::Mat *stencil;
};


#include <cstring>

#define eigen2mat4x4(eigm, mat) \
{\
float *__m__ = &mat[0][0];\
memcpy (__m__, eigm.data(), sizeof(float)*16);\
}


#define eigen2mat3x3(eigm, mat) \
{\
float *__m__ = &mat[0][0];\
memcpy (__m__, eigm.data(), sizeof(float)*9);\
}



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




#endif /* RENDERWIDGET_H_ */
