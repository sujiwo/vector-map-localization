#include "PointSolver2.h"
#include <string>
#include <vector>
#include <iostream>
#include "debug.h"
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

//	Point3 eyePos (-0.915031, -0.943627, 1.656656);
//	Quaternion eyeDir (0.985495, -0.117826, 0.121295, -0.014295);
	Point3 eyePos (0.5, 0.5, 2),
		centerOfView (0.5, 0.5, -2);
	Vector3 up (0, 1, 0);
	PointSolver2::Projector projector (45.0, 640, 480);

	// For testing projection
//	cv::Mat image;
//	PointSolver2::projectModel (image, model, projector, eyePos, centerOfView, up);
//	cv::imwrite ("/tmp/norm.png", image);

	cv::Mat timage = cv::imread ("/tmp/test.png", CV_LOAD_IMAGE_GRAYSCALE);

	PointSolver2 solver (model, projector);

	// initial pose
	Matrix4 vm = createViewMatrix (eyePos, centerOfView, up);
	Quaternion eyeDir;
	poseFromViewMatrix (vm, eyePos, eyeDir);
	solver.solve (timage, eyePos, eyeDir);



}
