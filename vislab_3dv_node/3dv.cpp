/* example/example2.h
 *
 * Copyright (C) 2013 VisLab
 *
 * This file is part of lib3dv; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include<lib3dv/utils.h>
#include<lib3dv/device.h>
#include <lib3dv/ipm.h>

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <deque>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"
/**
  * @brief This basic example shows how to convert disparity image into 3D metric coordinates .
  * 
 */
ros::Publisher pub;


class image_processor
{
  
public:
  
    image_processor(lib3dv::calibration::camera_params params) : m_ipm(params.m_intrinsics,params.m_position,params.m_orientation,params.m_baseline)
    {
         m_thread = boost::thread (boost::bind(&image_processor::update, this));
	 m_point_state=0;
    }
    
    ~image_processor()
    {
         m_thread.interrupt();
         m_thread.join();
    }
  
    void image_callback(boost::shared_ptr<const lib3dv::image> image)
    {
         /*DO NOT put your code here*/
         //This function is executed in the network handling thread, so it should not perform any actual processing.
         //Instead, it should just copy over the provided shared pointer and notify a worker thread.
         
         //The following code lines copy the shared pointers to the shared container.
         //if the worker thread is slower than acquisition thread, images will not be lost but the process memory will continue to increase.
      
         boost::lock_guard<boost::mutex> lock(m_mut);
         m_images.push_back(image);
         m_cond.notify_one();
    }
    
    void process_data(boost::shared_ptr<const lib3dv::image> image)
    {
      static int count;
      /* Put your code here*/
         // This function is executed  is executed in the worker thread
      
 
      if(m_point_state==0){
	out_points.clear();
      }
      if (image->type() == lib3dv::image::type::RIGHT_RECTIFIED)
	{
	  m_point_state=1;
	  m_image.resize(image->width()*image->height());
	  printf("%d\n",image->width()*image->height());
	  //m_image = *image;
	  for(int i=0;i<image->width()*image->height();i++)
	    m_image[i]=*((unsigned char*)image->data()+i);
	  m_width=image->width();
	  m_height=image->height();
	}
      if (image->type() == lib3dv::image::type::DSI)
	{    
	  if(m_point_state!=1){
	    m_point_state=0;
	    return;
	  }
	  m_point_state=2;
	  //	  out_points.header=msg->header;
	  count++;
	  world_points.clear();
	  m_ipm(image, world_points);
	  printf("%d %d\n",image->width()*image->height(), world_points.size());
	  pcl::PointXYZRGB pt;
	  //image_data=(unsigned char *)m_image.data();
	  for(int i=0;i<image->width();i++){
	    for(int j=0;j<image->height();j++){
	      if(world_points[i+j*image->width()].m_x<100){
		pt.x = world_points[i+j*image->width()].m_x;
		pt.y = world_points[i+j*image->width()].m_y;
		pt.z = world_points[i+j*image->width()].m_z;
		pt.r = m_image[m_width*j+i];
		pt.g = m_image[m_width*j+i];
		pt.b = m_image[m_width*j+i];
		out_points.push_back(pt);
	      }
	    }	    
	    //std::cout<<"."<<std::flush;           
	  }
	}
      if(m_point_state>=2){
	out_points.header.stamp = pcl_conversions::toPCL(ros::Time::now());
	out_points.header.frame_id = "vislab3dv";
	out_points.header.seq=count;
	pub.publish(out_points);
	out_points.clear();
	m_point_state=0;
      }
    }
  
private:
  
    void update()
    {
      
        boost::shared_ptr<const lib3dv::image> image;
        while (1)
        {       
            {
                boost::unique_lock<boost::mutex> lock(m_mut);
                while(m_images.empty())
                {
                    m_cond.wait(lock);
                }
                image = m_images.front();
                m_images.pop_front();
            }
            process_data(image);
        }
    }  

  std::vector<unsigned char > m_image;
  int m_width;
  int m_height;
  boost::condition_variable m_cond;
    boost::mutex m_mut;
    boost::thread m_thread;
    std::deque< boost::shared_ptr<const lib3dv::image> > m_images;
    lib3dv::inverse_perspective_mapping m_ipm;
    std::vector<lib3dv::point3> world_points;
    FILE* gnuplot;
    FILE* range;
  //    lib3dv::image m_image;
    int m_point_state;
    pcl::PointCloud<pcl::PointXYZRGB> out_points;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_3dv"); 
  ros::NodeHandle n;

  pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("vislab3dv", 1);
//
//   //List available devices
//   //This function lists available devices  bound to local IP address. See the list_interface_addresses() function in 3dv-ui/src/utils.h to get all IP address
    std::vector<lib3dv::device> devices;
  
    std::vector<boost::asio::ip::address> addresses = lib3dv::list_interface_addresses();

    for(std::vector<boost::asio::ip::address>::const_iterator aa = addresses.begin(); aa != addresses.end(); ++aa)
    {
        try{
            std::vector<lib3dv::device> dev = lib3dv::device::enumerate(aa->to_v4(), 0);
            devices.insert(devices.end(), dev.begin(), dev.end());
        }catch(...){}
    }

    if (devices.empty())
    {
        std::cout<<"No device found"<<std::endl;
        return 1;
    }    

    //I select the first device 
    lib3dv::device device = devices.front();
    
    lib3dv::calibration::camera_params camera_params = device.camera_params();

    image_processor proc(camera_params);

    //Register my image completion handler.
    //The supplied user-defined function will be called each time a full image is received.
    device.connect_image_callback(boost::bind(&image_processor::image_callback, &proc, _1));

    //Enable data transmission on the device
    device.start_transmission(false);

    ros::spin();
    //getchar();

    return 0;
}
