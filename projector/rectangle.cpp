/*
 * rectangle.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: sujiwo
 */

#include "SceneManager.h"
#include "PointListDrawObject.h"
#include "RectangleDrawObject.h"
#include "BitmapBackground.h"
#include "Rate.h"
#include <signal.h>


#define WIDTH 800
#define HEIGHT 600


void interrupt (int s)
{
	exit(1);
}



int main (int argc, char **argv)
{
	Rate loop (25);
	SceneManager &scene = SceneManager::getInstance(WIDTH, HEIGHT);
	signal (SIGINT, interrupt);

	PointListDrawObject::Ptr pList (new PointListDrawObject);
	pList->add (Point3 (0, 0, 0));
	pList->add (Point3 (1, 0, 0));
	pList->add (Point3 (1, 10, 0));
	pList->add (Point3 (0, 10, 0));
	pList->setColor(255, 255, 0);
	pList->setSize(6.0);

	//BitmapBackground::Ptr rect (new BitmapBackground(WIDTH, HEIGHT));

	//scene.addObject (rect);
	scene.addObject (pList);

	scene.getCamera()->lookAt (
		Point3(0.5, -1, 0.5),
		Point3(0.5, 1, 0.5),
		Vector3(0, 0, 1));
	scene.getCamera()->perspective (45.0,
		(float)WIDTH / (float)HEIGHT,
		1.0, 100);

	while (true) {
		scene.update();
		loop.sleep();
	}

	return 0;
}
