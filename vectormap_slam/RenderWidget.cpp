/*
 * RenderWidget.cpp
 *
 *  Created on: Jul 7, 2015
 *      Author: sujiwo
 */

#include "MatchWindow.h"
#include "RenderWidget.h"
#include <string>
#include "Math.h"
#include "tf_eigen.h"
#include <QtGui/QPainter>

#define GLDEBUG
#include "debug.h"


using std::string;


string defaultVertexShaderString (
"#version 110\n"
"uniform mat4 modelMat,\n"
"	viewMat,\n"
"	offsetMat,"
"	projectionMat;\n"
"\n"
"void main()\n"
"{ gl_Position = projectionMat * viewMat * modelMat * gl_Vertex; }\n"
);

string defaultFragmentShaderString (
"#version 110\n"
"uniform vec4 objColor;"
"\n"
"void main() \n"
"{ gl_FragColor = objColor; }\n"
);



RenderWidget::RenderWidget (int wWidth, int wHeight, QWidget *parent) :
	QGLWidget (QGLFormat(QGL::DoubleBuffer), parent),
	renderBuffer (NULL),
	realsize (wWidth, wHeight),
	stencil (NULL)
{
	camera = new Camera (wWidth, wHeight);
	setFixedSize (1, 1);
	makeCurrent ();
}


RenderWidget::~RenderWidget()
{
	delete (shader);
	delete (camera);
	for (VectorMapObject* obj : drawObjects) {
		delete (obj);
	}
	if (renderBuffer)
		delete (renderBuffer);
}


void RenderWidget::initializeGL ()
{
	// Framebuffer setup
	QGLFramebufferObjectFormat fbformat;
	fbformat.setAttachment(QGLFramebufferObject::CombinedDepthStencil);
	fbformat.setSamples(4);
	renderBuffer = new QGLFramebufferObject (realsize, fbformat);
	glViewport (0, 0, (GLsizei)realsize.width(), (GLsizei)realsize.height());

	shader = new QGLShaderProgram();
	shader->addShaderFromSourceCode (QGLShader::Vertex, defaultVertexShaderString.c_str());
	shader->addShaderFromSourceCode (QGLShader::Fragment, defaultFragmentShaderString.c_str());
	shader->link();
	shader->bind();

	qglClearColor(QColor(0, 0, 0, 0));

	for (VectorMapObject* obj : drawObjects) {
		obj->initialize();
	}

	if (stencil != NULL) {
//		glColorMask (GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
//		glStencilFunc (GL_ALWAYS, 2, ~0);
//		glStencilOp (GL_REPLACE, GL_REPLACE, GL_REPLACE);
//
//		QPainter stencilDraw (renderBuffer);
//		QPoint o (0, 0);
//		stencilDraw.drawImage (o, *stencil);
//
		glEnable (GL_STENCIL_TEST);
		glDrawPixels (realsize.width(), realsize.height(), GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, stencil->data);
		glStencilFunc (GL_EQUAL, 0xff, 0xff);
		glStencilOp (GL_KEEP, GL_KEEP, GL_KEEP);
	}
}


void RenderWidget::paintGL ()
{
	return render ();
}


void RenderWidget::render ()
{
	renderBuffer->bind();
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	cameraToShader ();
	for (VectorMapObject* obj : drawObjects) {
		obj->draw();
	}

	glFlush ();
	renderBuffer->release();
}


void RenderWidget::cameraToShader ()
{
	return cameraToShader (shader);
}


//Matrix4 offsetPose2Eigen (const Point3 &offPos, const Quaternion &offOri)
//{
//	Matrix4 offsetMat = Matrix4::Identity();
//	Quaternion orin;
//	orin = offOri * Eigen::Quaternionf( Eigen::AngleAxisf(degreeToRadian(180), Eigen::Vector3f::UnitX()) );
//	orin.normalize();
//
//	offsetMat.block<3,3> (0,0) = orin.toRotationMatrix().transpose();
//	offsetMat.block<3,1> (0,3) = offsetMat.block<3,3> (0,0) * (-offPos);
//
//	return offsetMat;
//}


void RenderWidget::cameraToShader (QGLShaderProgram *shaderPrg)
{
	Matrix4 ident = Matrix4::Identity ();
	GLfloat m[4][4];
	eigen2mat4x4 (ident, m);
	shaderPrg->setUniformValue ("modelMat", m);
	eigen2mat4x4 (camera->getViewMatrix(), m);
	shaderPrg->setUniformValue ("viewMat", m);
	eigen2mat4x4 (camera->getProjectionMatrix(), m);
	shaderPrg->setUniformValue ("projectionMat", m);

	return;
}


void RenderWidget::update()
{
	makeCurrent ();
	render ();
}


void RenderWidget::draw (bool useOffset)
{
	tf::Transform trf;

	if (useOffset) {
		Quaternion absOri;
		Point3 absPos;
		mypose.getAbsolute (absPos, absOri);
		trf = eigen2tf (absOri, absPos);
	}
	else {
		trf = eigen2tf (mypose.getOrientation(), mypose.getPosition());
	}
	camera->lookAt (trf);
	update ();
}


cv::Mat RenderWidget::convert2Cv (QImage *isrc)
{
	int mattype;
	switch (isrc->format ()) {
		case QImage::Format_RGB32:
			mattype = CV_8UC4; break;
		case QImage::Format_ARGB32:
			mattype = CV_8UC4; break;
		case QImage::Format_ARGB32_Premultiplied:
			mattype = CV_8UC4; break;
		case QImage::Format_RGB888:
			mattype = CV_8UC3; break;
		case QImage::Format_Indexed8:
			if (isrc->isGrayscale())
				mattype = CV_8UC1;
			else throw "Unknown image";
	}

	cv::Mat imageDst (isrc->height(), isrc->width(), mattype);
	for (int r=0; r<isrc->height(); r++) {
		memcpy (imageDst.ptr(r), isrc->scanLine(r), isrc->bytesPerLine());
	}

	return imageDst;
}


void RenderWidget::getImage (cv::Mat &cvmat)
{
	QImage pbuf = this->getImage();
	cvmat = RenderWidget::convert2Cv(&pbuf);
}


TGLWidget::TGLWidget(QWidget *parent) :
	QGLWidget (QGLFormat(QGL::SampleBuffers), parent)
{}


TGLWidget::~TGLWidget()
{}


void TGLWidget::initializeGL ()
{}


void TGLWidget::paintGL ()
{
	glwrap(glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT))
}


void TGLWidget::resizeGL ()
{}


void RenderWidget::setStencil(cv::Mat *stencilImage)
{
	stencil = stencilImage;
}
