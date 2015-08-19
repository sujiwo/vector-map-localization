/*
 * SceneManager.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: sujiwo
 */

#include "SceneManager.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/freeglut.h>
#include <string>
#include <cstdio>


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
"{ gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0); }\n"
);




SceneManager::SceneManager (int w, int h) :
	windowWidth (w),
	windowHeight (h),
	positionOffset (tf::Vector3(0, 0, 0))
{
	camera = new Camera (w, h);
	int argc = 1;
	glutInit (&argc, NULL);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (windowWidth, windowHeight);
	glutInitWindowPosition (10, 10);
	glutCreateWindow ("Unknown");
	glewInit ();

	glutDisplayFunc (SceneManager::display);
	glClearColor (0.0, 0.0, 0.0, 0.0);

	shader = ShaderProgram::fromString (defaultVertexShaderString, defaultFragmentShaderString);
	framebuffer = new Framebuffer (windowWidth, windowHeight);
	defaultFramebuffer = Framebuffer::getDefault();

	glutKeyboardFunc (SceneManager::keyboardFunc);
}


SceneManager::~SceneManager()
{
	delete (camera);
	delete (framebuffer);
}


SceneManager& SceneManager::getInstance (int windowWidth, int windowHeight)
{
	static SceneManager *inst = NULL;

	if (inst==0) {
		inst = new SceneManager (windowWidth, windowHeight);
	}
	return *inst;
}


void SceneManager::display()
{
	SceneManager &s = getInstance ();
	s.update();
}


void SceneManager::update2 ()
{
	Matrix4 m = Matrix4::Identity();
	shader.setUniform ("modelMat", UNIFORM_MAT4F, &m);
	shader.setUniform ("viewMat", UNIFORM_MAT4F, &camera->getViewMatrix());
	shader.setUniform ("projectionMat", UNIFORM_MAT4F, &camera->getProjectionMatrix());
	DrawObject::Ptr obj;
	for (int i=0; i<drawableList.size(); i++) {
		obj = drawableList [i];
		obj->draw();
	}
	glFlush();
	glutSwapBuffers ();
	printf ("Updated\n");
}


void SceneManager::update()
{
	// camera setting
	Matrix4 m = Matrix4::Identity();
	shader.setUniform ("modelMat", UNIFORM_MAT4F, &m);
	shader.setUniform ("viewMat", UNIFORM_MAT4F, &camera->getViewMatrix());
	shader.setUniform ("projectionMat", UNIFORM_MAT4F, &camera->getProjectionMatrix());

	// set-up framebuffer to draw to renderbuffer
	framebuffer->useForDrawing();
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw each object separately
	DrawObject::Ptr obj;
	for (int i=0; i<drawableList.size(); i++) {
		obj = drawableList [i];
		obj->draw();
	}
	glFlush();

	// From this point, rendering is finished. The rest of this function
	// is to send render result into window framebuffer

	// set-up to read from renderbuffer and draw to window framebuffer
	framebuffer->useForReading();
	//Framebuffer::getDefault()->useForDrawing();
	glBindFramebuffer (GL_DRAW_FRAMEBUFFER, 0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBlitFramebuffer (0, 0, windowWidth-1, windowHeight-1,
		0, 0, windowWidth-1, windowHeight-1,
		GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glutSwapBuffers ();
}


void SceneManager::loopEvent()
{
	glutMainLoopEvent ();
}


void SceneManager::addObject (DrawObject::Ptr obj)
{
	drawableList.push_back (obj);
	obj->initialize();
	obj->_setScene (this);
}


void SceneManager::setPose (const tf::Transform &tnf)
{
	currentPose = tnf;
	tf::Vector3 synPos = tnf.getOrigin() + this->positionOffset;
	currentPose.setOrigin(synPos);
	camera->lookAt (currentPose);
}


void SceneManager::setPose ()
{
	tf::Vector3 neworigin = currentPose.getOrigin() + this->positionOffset;
	currentPose.setOrigin (neworigin);
	camera->lookAt (currentPose);
}


void SceneManager::setPose (const Point3 &position, const Quaternion &orientation)
{
	//currentPose.setOrigin ()
}



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


void SceneManager::keyboardFunc (unsigned char k, int x, int y)
{
	SceneManager &scn = SceneManager::getInstance();
	if (k=='c' || k=='K')
		return scn.captureToDisk ();
	if (k=='a')
		return scn.offsetPos(-0.1, 0, 0);
	if (k=='d')
		return scn.offsetPos(0.1, 0, 0);
	if (k=='w')
		return scn.offsetPos(0, -0.1, 0);
	if (k=='s')
		return scn.offsetPos(0, 0.1, 0);
	//printf ("Key: %d\n", k);

	switch (k) {
	case 'A': return scn.offsetRotation(0.1, 0, 0); break;
	}
}


void SceneManager::captureToDisk()
{
	printf ("PosX:%f, PosY:%f, PosZ:%f\n",
		currentPose.getOrigin().x(),
		currentPose.getOrigin().y(),
		currentPose.getOrigin().z());
	printf ("OrientX:%f, OrientY:%f, OrientZ:%f, OrientW:%f\n",
		currentPose.getRotation().x(),
		currentPose.getRotation().y(),
		currentPose.getRotation().z(),
		currentPose.getRotation().w());

	// save current framebuffer to disk
	// XXX: Direct image from OpenGL is mirrored horizontally, rotated 180 deg
	uint8_t *mem = new uint8_t[framebuffer->getSize()];
	framebuffer->copyTo(mem);
	cv::Mat bmp (framebuffer->height(), framebuffer->width(), CV_8UC3, mem);
	cv::flip (bmp, bmp, -1);
	cv::flip (bmp, bmp, 1);
	cv::Mat grey;
	cv::cvtColor (bmp, grey, CV_RGB2GRAY);
	cv::imwrite ("/tmp/synthetic.png", grey);
	delete[] mem;

	// save camera image to disk, in grayscale
	cv::Mat camImg (framebuffer->height(), framebuffer->width(), CV_8UC3, cameraBackground->getBuffer());
	cv::flip (camImg, camImg, -1);
	cv::flip (camImg, camImg, 1);
	cv::cvtColor (camImg, grey, CV_RGB2GRAY);
	cv::imwrite ("/tmp/real.png", grey);
}


void SceneManager::offsetPos(float dx, float dy, float dz)
{
	this->positionOffset += tf::Vector3 (dx, dy, dz);
	this->setPose ();
	this->update ();
	//std::cout << currentPose.getOrigin();
}


void SceneManager::offsetReset ()
{
	this->positionOffset = tf::Vector3 (0, 0, 0);
	this->setPose ();
	this->update ();
}


void SceneManager::offsetRotation (float roll, float pitch, float yaw)
{
	tf::Quaternion rot = currentPose.getRotation();
}
