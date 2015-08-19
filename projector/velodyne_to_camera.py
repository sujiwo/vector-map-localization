from __future__ import division
import numpy as np
import cv
import tf.transformations as tfm
import rospy
import tf
import sys
		
		
def loadSource (filename):
	content = cv.Load(filename)
	return np.asarray(content)

		

if __name__ == '__main__' :
	# Prepare transformation
	filename = sys.argv[1]
	ymsrc = loadSource(filename)
	print ("Loaded "+filename)
	quaternion = tfm.quaternion_from_matrix(ymsrc)
	translation = (ymsrc[0,3], ymsrc[1,3], ymsrc[2,3])
	
	rospy.init_node ("velodyne_to_camera_publisher")
	sender = tf.TransformBroadcaster (100)
	rate = rospy.Rate (25)
	
	while not rospy.is_shutdown() :
		sender.sendTransform(translation, quaternion, rospy.get_rostime(), "camera", "velodyne")
		print ("Sent")
		rate.sleep()
