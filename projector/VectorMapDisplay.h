#include "SceneManager.h"
#include "PointListDrawObject.h"
#include "SignalsDrawObject.h"
#include "LineListDrawObject.h"
#include "LaneListDrawObject.h"
#include "vector_map.h"
#include <string>


using std::string;


class VectorMapDisplay
{
public:
	VectorMapDisplay (const string &mapdir, int _width, int _height);
	~VectorMapDisplay ();

private:
	VectorMap vectormap;
	int width, height;
	SceneManager &scene;
	SignalsDrawObject::Ptr signals;
};
