/*
 * SceneManager.h
 *
 *  Created on: Apr 8, 2015
 *      Author: sujiwo
 */

#ifndef SRC_SCENEMANAGER_H_
#define SRC_SCENEMANAGER_H_


#include <vector>
#include "Math.h"
#include "ShaderProgram.h"
#include "Camera.h"
#include "DrawObject.h"
#include "Framebuffer.h"
#include "CameraImageBackground.h"


using std::vector;


class SceneManager
{
public:

	SceneManager (int windowWidth, int windowHeight);
	virtual ~SceneManager();
	static SceneManager& getInstance (int windowWidth=-1, int windowHeight=-1);

	ShaderProgram* getShader () { return &shader; }
	Camera* getCamera () { return camera; }
	Framebuffer *getMainFramebuffer () { return framebuffer; }

	static void display ();
	void update ();
	void update2 ();
	void loopEvent ();

	void addObject (DrawObject::Ptr obj);

	void setPose (const tf::Transform &tnf);
	void setPose (const Point3 &position, const Quaternion &orientation);
	void setPose ();

	static void keyboardFunc (unsigned char key, int x, int y);

	void setCameraBackground (CameraImageBackground::Ptr cam)
	{ cameraBackground = cam; }

	void captureToDisk ();
	void offsetPos (float dx, float dy, float dz);
	void offsetReset ();
	void offsetRotation (float roll, float pitch, float yaw);

private:
	ShaderProgram shader;
	Camera *camera;
	Framebuffer *framebuffer,
		*defaultFramebuffer;

	vector<DrawObject::Ptr> drawableList;

	tf::Transform currentPose;
	tf::Vector3 positionOffset;

	int windowWidth, windowHeight;
	CameraImageBackground::Ptr cameraBackground;
};

#endif /* SRC_SCENEMANAGER_H_ */
