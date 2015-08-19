#include "Math.h"
#include "Rate.h"
#include <signal.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <sqlite3.h>
#include <vector>



tf::StampedTransform transform;
image_transport::ImageTransport *imgtrans;
image_transport::Subscriber camSub;
//cv_bridge::CvImageConstPtr imageBkg;
sensor_msgs::Image::ConstPtr imageBkg;
bool isNewImage = false;
sqlite3 *bagtable = NULL;
sqlite3_stmt *bagIns = NULL;



void getTransform ()
{
	static tf::TransformListener listener;

	// target_frame    source_frame
	listener.waitForTransform ("camera", "japan_7", ros::Time(), ros::Duration(10.0));
	listener.lookupTransform ("camera", "japan_7", ros::Time(), transform);
}


void cleanUp ()
{
	sqlite3_finalize(bagIns);
	sqlite3_close(bagtable);
}


void interrupt (int s)
{
	cleanUp ();
	exit(1);
}


void echoMsg ()
{
	int w, h;
	tf::Vector3 &p = transform.getOrigin();
	tf::Quaternion o = transform.getRotation();

	if (isNewImage) {
		w = imageBkg->width, h = imageBkg->height;
		isNewImage = false;

		cv_bridge::CvImagePtr imagecopy = cv_bridge::toCvCopy(imageBkg, "rgb8");
		// encode image
		bool r;
		std::vector<uint8_t> imgbuf;
		r = cv::imencode(".jpg", imagecopy->image, imgbuf);

		sqlite3_reset (bagIns);
		sqlite3_bind_double(bagIns, 1, imageBkg->header.stamp.toSec());
		sqlite3_bind_double(bagIns, 2, p.x());
		sqlite3_bind_double(bagIns, 3, p.y());
		sqlite3_bind_double(bagIns, 4, p.z());
		sqlite3_bind_double(bagIns, 5, o.x());
		sqlite3_bind_double(bagIns, 6, o.y());
		sqlite3_bind_double(bagIns, 7, o.z());
		sqlite3_bind_double(bagIns, 8, o.w());
		sqlite3_bind_blob (bagIns, 9, imgbuf.data(), imgbuf.size(), SQLITE_STATIC);
		int s = sqlite3_step (bagIns);
		if (s != SQLITE_DONE) {
			const char *sqErrMsg = sqlite3_errstr(s);
			printf ("Unable to execute sqlite: %s\n", sqErrMsg);
		}
	}
	else {
		w = h = 0;
	}

//	printf ("%f %f %f, %f %f %f %f, (%d %d)\n",
//		p.x(), p.y(), p.z(),
//		o.w(), o.x(), o.y(), o.z(),
//		w, h);
}


void imageCatch (const sensor_msgs::Image::ConstPtr &imagemsg)
{
	imageBkg = imagemsg;
	isNewImage = true;
}


int main (int argc, char *argv[])
{
	// ROS business
	ros::init(argc, argv, "grab", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;
	
	imgtrans = new image_transport::ImageTransport (rosnode);
	camSub = imgtrans->subscribe ("/camera/image_raw", 10, imageCatch);

	signal (SIGINT, interrupt);
	Rate loop (25);
	
	// table
	sqlite3_open (argv[1], &bagtable);
	sqlite3_prepare (bagtable,
		"INSERT INTO bags(tstamp,px,py,pz,rx,ry,rz,rw,image) VALUES(?,?,?,?,?,?,?,?,?)",
		-1, &bagIns, NULL);

	while (true) {

		ros::spinOnce();

		try {
			getTransform ();
		} catch (tf::TransformException &exc) {
		}
		
		echoMsg ();

		loop.sleep();
	}

	return 0;

}
