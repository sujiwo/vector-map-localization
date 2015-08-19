/*
 * VectorMapRenderWidget.cpp
 *
 *  Created on: Jul 7, 2015
 *      Author: sujiwo
 */

#include "VectorMapRenderWidget.h"
#include <string>
#include "Math.h"
#include "debug.h"


using std::string;


string defaultVertexShaderString (
"#version 110\n"
"uniform mat4 modelMat,\n"
"	viewMat,\n"
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



VectorMapRenderWidget::VectorMapRenderWidget(QWidget *parent) :
	QGLViewer(parent),
	camera (NULL),
	hasInit (false)
{}


VectorMapRenderWidget::~VectorMapRenderWidget()
{
	// TODO Auto-generated destructor stub
	delete (shader);
	for (VectorMapObject* obj : drawObjects) {
		delete (obj);
	}
}


void VectorMapRenderWidget::setSize (int width, int height)
{
	setFixedSize (width, height);
	hasInit = true;
}


void VectorMapRenderWidget::initializeGL ()
{
	if (!camera)
		camera = new Camera (this->width(), this->height());

	camera->lookAt (
		Point3 (0, -1, 1),
		Point3 (0, 1, 1),
		Vector3 (0, 0, 0)
	);
	camera->perspective(45.0, (float)camera->getWidth()/(float)camera->getHeight(), 1.5, 6);

	shader = new QGLShaderProgram();
	shader->addShaderFromSourceCode (QGLShader::Vertex, defaultVertexShaderString.c_str());
	shader->addShaderFromSourceCode (QGLShader::Fragment, defaultFragmentShaderString.c_str());
	shader->link();
	shader->bind();

	for (VectorMapObject* obj : drawObjects) {
		obj->initialize();
	}
}


void VectorMapRenderWidget::resizeGL (int w, int h)
{
	glViewport (0, 0, (GLsizei)w, (GLsizei)h);
	camera->setSize(w, h);
}


void VectorMapRenderWidget::paintGL ()
{
	if (hasInit==false)
		return;

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	cameraToShader ();
	for (VectorMapObject* obj : drawObjects) {
		obj->draw();
	}
	glFlush ();
}


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



void VectorMapRenderWidget::cameraToShader ()
{
	int mmid = shader->uniformLocation("objColor");
	Matrix4 ident = Matrix4::Identity ();
	GLfloat m[4][4];
	eigen2mat4x4 (ident, m);
	shader->setUniformValue ("modelMat", m);
	eigen2mat4x4 (camera->getViewMatrix(), m);
	shader->setUniformValue ("viewMat", m);
	eigen2mat4x4 (camera->getProjectionMatrix(), m);
	shader->setUniformValue ("projectionMat", m);
	return;
}


void VectorMapRenderWidget::update()
{ if (hasInit==true) return paintGL(); }


void VectorMapRenderWidget::setPose (double position[3], double orientation[4])
{
	// We directly create view matrix here
//	Vector3 camPos (position[0], position[1], position[2]);
//	Matrix3 camRot;
//	Matrix4 viewMat = Matrix4::Identity();
//	Quaternion camRotq (orientation[0], orientation[1], orientation[2], orientation[3]);
//	camRot = camRotq.toRotationMatrix();
//	viewMat.block<3, 3>(0, 0) = camRot.transpose();
//	viewMat.block<3, 1>(0, 3) = -camRot.transpose() * camPos;

//	Matrix4 rotfix = Matrix4::Identity();
//	rotfix (1,1) = -1;
//	rotfix (2,2) = -1;
//	rotfix (3,3) = 1;
//	viewMat = rotfix * viewMat;

//	camera->setViewMatrix (viewMat);
	Point3 curPos (position[0], position[1], position[2]);
	Point3 target = curPos + Vector3 (0, 1, 0);
	camera->lookAt (
		curPos,
		target,
		Vector3 (0, 0, 1)
	);
	Matrix4 &viewmat = camera->getViewMatrix();
	std::cout << viewmat << std::endl;
}


TGLWidget::TGLWidget(QWidget *parent) :
	QGLWidget (QGLFormat(QGL::SampleBuffers), parent)
{}


TGLWidget::~TGLWidget()
{}


void TGLWidget::initializeGL ()
{}


void TGLWidget::paintGL ()
{}


void TGLWidget::resizeGL ()
{}

