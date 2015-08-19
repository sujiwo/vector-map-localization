#include "VectorMapDisplay.h"


VectorMapDisplay::VectorMapDisplay (const string &mapdir, int _width, int _height) :
	scene (SceneManager::getInstance(_width, _height))
{
	vectormap.loadAll (mapdir);
	signals = SignalsDrawObject::Ptr(new SignalsDrawObject (vectormap));

	scene.addObject (signals);
}


VectorMapDisplay::~VectorMapDisplay ()
{

}
