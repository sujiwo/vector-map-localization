#include <lib3dv/utils.h>
#include <lib3dv/device.h>
#include <lib3dv/ipm.h>
#include <iostream>
#include <vector>
#include <queue>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdint.h>
//#include <libdsi/dsi_export.h>
#include <libdsi/disparity_filter.h>



using std::vector;
using std::cout;
using std::endl;
using std::queue;


class ImageHandler
{
public:
	ImageHandler (ros::NodeHandle &n, lib3dv::device &dev) :
		imageTrn (n),
		device (dev),
		mPointState (0),
		cameraParams (device.camera_params()),
		mIpm (cameraParams.m_intrinsics,
			cameraParams.m_position,
			cameraParams.m_orientation,
			cameraParams.m_baseline)
	{
		imageRightPub = imageTrn.advertise("/vislab/right", 1);
		depthMapPub = imageTrn.advertise ("/vislab/depth", 1);
		pointCloudPub = n.advertise <pcl::PointCloud<pcl::PointXYZI> > ("/vislab/pointcloud", 1);
		mythread = boost::thread (boost::bind(&ImageHandler::threadStart, this));
		device.connect_image_callback(boost::bind(&ImageHandler::callback, this, _1));
	}


	~ImageHandler ()
	{
		mythread.interrupt();
		mythread.join();
	}


	void callback (boost::shared_ptr<const lib3dv::image> image)
	{
		boost::lock_guard<boost::mutex> lock (mymutex);
		imageqs.push (image);
		mycond.notify_one();
	}


private:
	image_transport::ImageTransport imageTrn;
	image_transport::Publisher imageRightPub;
	image_transport::Publisher depthMapPub;
	lib3dv::device &device;
	boost::thread mythread;
	boost::condition_variable mycond;
	boost::mutex mymutex;
	queue<boost::shared_ptr<const lib3dv::image> > imageqs;
	int mPointState;
	lib3dv::calibration::camera_params cameraParams;
	lib3dv::inverse_perspective_mapping mIpm;
	vector<lib3dv::point3> worldPoints;
	ros::Publisher pointCloudPub;


	void publish (boost::shared_ptr<const lib3dv::image> image, lib3dv::image::type::types type)
	{
		static int counterRight = 0,
				counterBsi = 0;
		int step, counter;
		std::string encoding;

		if (type==lib3dv::image::type::RIGHT_RECTIFIED) {
			encoding = sensor_msgs::image_encodings::MONO8;
			step = image->width();
			counter = counterRight++;
		}
		else if (type==lib3dv::image::type::DSI) {
			encoding = sensor_msgs::image_encodings::MONO16;
			step = image->width()*2;
			counter = counterBsi++;
		}

		sensor_msgs::Image newimg;
		newimg.header.stamp = ros::Time::now();
		newimg.header.seq = counter;
		sensor_msgs::fillImage (newimg,
			encoding,
			image->height(), image->width(),
			step,
			image->data());

		if (type==lib3dv::image::type::RIGHT_RECTIFIED) {
			imageRightPub.publish (newimg);
		}
		else if (type==lib3dv::image::type::DSI) {
			depthMapPub.publish (newimg);
		}
	}


#define CUTRANGE 10.0

	void publishPointCloud (boost::shared_ptr<const lib3dv::image> image)
	{
		static int counter = 0;
		pcl::PointCloud <pcl::PointXYZI> pointsOut;
		uint16_t *imgdata = (uint16_t*)image->data();

		// do filtering
		//filling_filter (imgdata, image->width(), image->height(), imgdata, 3, 2);
		despeckle_filter (imgdata, image->width(), image->height(), imgdata, 1, 25, 2);

		worldPoints.clear();
		mIpm (image, worldPoints);

		for (int i=0; i<image->width(); i++) {
			for (int j=0; j<image->height(); j++) {
				lib3dv::point3 &pts = worldPoints[j*image->width() + i];

				if (pts.m_x < CUTRANGE) {
					pcl::PointXYZI point;
					point.x = pts.m_x;
					point.y = pts.m_y;
					point.z = pts.m_z;
					// Change this
					uint16_t ints = imgdata[j*image->width() + i];
					point.intensity = (float)ints / 0xFFFF;
					//point.intensity = 1.0;
					pointsOut.push_back(point);
				}
			}
		}

		int size = pointsOut.size();
		pointsOut.header.seq = counter++;
		ros::Time t = ros::Time::now();
		pcl_conversions::fromPCL(pointsOut.header.stamp, t);
		pointsOut.header.frame_id = "vislab3dv";
		pointCloudPub.publish (pointsOut);
		pointsOut.clear();
	}


	void process (boost::shared_ptr<const lib3dv::image> image)
	{
		static int count;

		publish (image, image->type());
		if (image->type()==lib3dv::image::type::DSI) {
			publishPointCloud (image);
		}
	}


	void threadStart ()
	{
		boost::shared_ptr<const lib3dv::image> image;
		while (1) {
			{
				boost::unique_lock <boost::mutex> lock (mymutex);
				while (imageqs.empty()) {
					mycond.wait (lock);
				}
				image = imageqs.front();
				imageqs.pop();
			}
			process (image);
		}
	}
};




int main (int argc, char *argv[])
{
	// 3DV device enumeration
	vector<lib3dv::device> devices;
	vector<boost::asio::ip::address> addresses = lib3dv::list_interface_addresses();

	for(std::vector<boost::asio::ip::address>::const_iterator aa = addresses.begin(); aa != addresses.end(); ++aa)
	{
		try{
			vector<lib3dv::device> dev = lib3dv::device::enumerate(aa->to_v4(), 0);
			devices.insert(devices.end(), dev.begin(), dev.end());
		} catch(...){}
	}

	if (devices.empty())
	{
		cout<<"No device found"<<endl;
		return 1;
	}

	//I select the first device
	lib3dv::device device = devices.front();

	// ROS Business
	ros::init(argc, argv, "vislab_3dv", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;

	ImageHandler imageraw (rosnode, device);
	device.start_transmission();

	ros::spin();

	return 0;
}
