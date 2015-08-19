#include "VectorMapDisplay.h"
#include <sqlite3.h>
#include <string>
#include <exception>
#include <vector>
#include <Math.h>
#include "ImageDB.h"
#include "dbReadApp.h"



int main (int argc, char *argv[])
{
	ImageDB rosbag (argv[1]);
	Point3 position;
	Quaternion orientation;
	cv::Mat image;

	dbReadApp reader (&rosbag, argc, argv);
	reader.exec ();

	return 0;
}
