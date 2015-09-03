#include "PointSolver2.h"
#include <string>
#include <vector>
#include <iostream>
#include "debug.h"
#include "Camera.h"
#include "Math.h"


using std::vector;

vector<ModelLine> model;
Point3 rectangle[] = {
	Point3 (0, 0, -2),
	Point3 (1, 0, -2),
	Point3 (1, 1, -2),
	Point3 (0, 1, -2)
};


void buildModel (vector<ModelLine> &dataset, Point3 *points, int numOfPt)
{
	dataset.clear();
	for (int i=0; i<numOfPt-1; i++) {
		ModelLine line (points[i], points[i+1]);
		dataset.push_back(line);
	}
	ModelLine lcon (points[numOfPt-1], points[0]);
	dataset.push_back(lcon);
}


void buildModel (vector<ModelLine> &dataset, vector<Point3> &pointList)
{
	return buildModel (dataset, pointList.data(), pointList.size());
}


int main (int argc, char **argv)
{
	buildModel (model, rectangle, 4);

	// XXX: principal point cx and cy is undefined !
	Camera camera (640, 480);
	Point3 eyePos (-0.915031, -0.943627, 1.656656);
	Quaternion eyeDir (0.985495, -0.117826, 0.121295, -0.014295);
	camera.lookAt (eyeDir, eyePos);
	camera.perspective(45.0, 1.333, 1.0, 100);

	cv::Mat timage = cv::imread ("/tmp/test.png", CV_LOAD_IMAGE_GRAYSCALE);
	PointSolver2 solver (model, &camera, timage.rows, timage.cols);
	solver.solve (timage, eyePos, eyeDir);
//	PointSolver2::projectModel(timage, model, &camera, 640, 480);
//	cv::imwrite ("/tmp/modelp.png", timage);

	std::cout << camera.getProjectionMatrix() << std::endl;

//	Point3 eyePos;
//	Quaternion eyeDir;
//	camera.getPose(eyePos, eyeDir);

//	PointSolver2 solver2 (model, &camera, 640, 480);
//	solver2.solve ()
}
