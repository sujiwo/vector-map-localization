import sys
from math import floor, sqrt, pow, sin, cos, tan, pi
import rospy
from nmea_msgs.msg import Sentence as NmeaSentence
from geometry_msgs.msg import PoseStamped
import tf




class GeoConvert :
	def __init__ (self, plane=7):
		if (plane==6):
			self.m_PLo = 2.373647782712   
			self.m_PLato= 0.628318530717  
		elif (plane==7):
			self.m_PLo = 2.39400995732
			self.m_PLato=  0.628318530717
		elif (plane==9) :
			self.m_PLo =  2.4405520707
			self.m_PLato =  0.628318530717
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		
	def convertNmeaLatLon (self, latitude, longitude, height):
		# latitude/longitude correction
		lad = floor(latitude/100.0)
		lat = latitude - lad*100.0
		lod = floor(longitude/100.0)
		lon = longitude - lod*100.0
		# To radian
		latr = (lad+lat/60.0) * pi / 180.0
		lonr = (lod+lon/60.0) * pi / 180.0
		
		return self.convertLLH2XYZ(latr, lonr, height)


	def convertLLH2XYZ (self, lat, lon, height):
		Pmo = 0.9999
	
		#WGS84 Parameters
		AW = 6378137.0			#Semimajor Axis
		FW = 1.0/298.257222101	#Geometrical flattening
	    
		Pe  = sqrt (2.0*FW - pow(FW,2))
		Pet = sqrt ( pow(Pe,2) / (1.0 - pow(Pe,2)) )
	    
		PA = 1.0 + 3.0/4.0*pow(Pe,2) + 45.0/64.0* pow(Pe,4) + 175.0/256.0*pow(Pe,6) \
			+ 11025.0/16384.0*pow(Pe,8) + 43659.0/65536.0*pow(Pe,10) + 693693.0/1048576.0*pow(Pe,12) \
			+ 19324305.0/29360128.0*pow(Pe,14) + 4927697775.0/7516192768.0*pow(Pe,16)
	    
		PB = 3.0/4.0*pow(Pe,2) + 15.0/16.0*pow(Pe,4) + 525.0/512.0*pow(Pe,6) + 2205.0/2048.0*pow(Pe,8) \
			+ 72765.0/65536.0*pow(Pe,10) + 297297.0/262144.0*pow(Pe,12) + 135270135.0/117440512.0*pow(Pe,14) \
			+ 547521975.0/469762048.0*pow(Pe,16)
	    
		PC = 15.0/64.0*pow(Pe,4) + 105.0/256.0*pow(Pe,6) + 2205.0/4096.0*pow(Pe,8) + 10395.0/16384.0*pow(Pe,10) \
			+ 1486485.0/2097152.0*pow(Pe,12) + 45090045.0/58720256.0*pow(Pe,14)+ 766530765.0/939524096.0*pow(Pe,16)
	    
		PD = 35.0/512.0*pow(Pe,6) + 315.0/2048.0*pow(Pe,8) + 31185.0/131072.0*pow(Pe,10) \
			+ 165165.0/524288.0*pow(Pe,12) + 45090045.0/117440512.0*pow(Pe,14) + 209053845.0/469762048.0*pow(Pe,16)
	    
		PE = 315.0/16384.0*pow(Pe,8) + 3465.0/65536.0*pow(Pe,10) + 99099.0/1048576.0*pow(Pe,12) + \
			4099095.0/29360128.0*pow(Pe,14) + 348423075.0/1879048192.0*pow(Pe,16)
	    
		PF = 693.0/131072.0*pow(Pe,10) + 9009.0/524288.0*pow(Pe,12) +  4099095.0/117440512.0*pow(Pe,14) \
			+ 26801775.0/469762048.0*pow(Pe,16)

		PG = 3003.0/2097152.0*pow(Pe,12) + 315315.0/58720256.0*pow(Pe,14) + 11486475.0/939524096.0*pow(Pe,16)
	    
		PH = 45045.0/117440512.0*pow(Pe,14) + 765765.0/469762048.0*pow(Pe,16)
	    
		PI = 765765.0/7516192768.0*pow(Pe,16); 
	    
		PB1 = AW * (1.0 - pow(Pe,2)) * PA
		PB2 = AW * (1.0 - pow(Pe,2)) * PB/-2.0
		PB3 = AW * (1.0 - pow(Pe,2)) * PC/4.0
		PB4 = AW * (1.0 - pow(Pe,2)) * PD/-6.0
		PB5 = AW * (1.0 - pow(Pe,2)) * PE/8.0
		PB6 = AW * (1.0 - pow(Pe,2)) * PF/-10.0
		PB7 = AW * (1.0 - pow(Pe,2)) * PG/12.0
		PB8 = AW * (1.0 - pow(Pe,2)) * PH/-14.0
		PB9 = AW * (1.0 - pow(Pe,2)) * PI/16.0
	    
		PS = PB1*lat + PB2*sin(2.0*lat) + PB3*sin(4.0*lat) + PB4*sin(6.0*lat) \
			+ PB5*sin(8.0*lat) + PB6*sin(10.0*lat) + PB7*sin(12.0*lat) + PB8*sin(14.0*lat) \
			+ PB9*sin(16.0*lat);
	    
		PSo = PB1*self.m_PLato + PB2*sin(2.0*self.m_PLato) + PB3*sin(4.0*self.m_PLato) + PB4*sin(6.0*self.m_PLato) \
			+ PB5*sin(8.0*self.m_PLato) + PB6*sin(10.0*self.m_PLato) + PB7*sin(12.0*self.m_PLato) + PB8*sin(14.0*self.m_PLato) \
			+ PB9*sin(16.0*self.m_PLato)
	    
		PDL = lon - self.m_PLo;
		Pt  = tan(lat)
		PW  = sqrt(1.0 - pow(Pe,2)*pow(sin(lat),2))
		PN  = AW / PW
		Pnn = sqrt( pow(Pet,2) * pow(cos(lat),2))
		#print (PDL, Pt, PW, PN, Pnn, lon, self.m_PLo)
	    
		m_x = ( (PS - PSo) + (1.0/2.0)*PN*pow(cos(lat),2.0)*Pt*pow(PDL,2.0) \
			+ (1.0/24.0) * PN * pow(cos(lat),4) * Pt * (5.0-pow(Pt,2) + 9.0*pow(Pnn,2) + 4.0*pow(Pnn,4))*pow(PDL,4) \
			- (1.0/720.0) * PN * pow(cos(lat),6) * Pt * \
			(-61.0 + 58.0*pow(Pt,2) - pow(Pt,4) - 270.0*pow(Pnn,2) + 330.0*pow(Pt,2)*pow(Pnn,2))*pow(PDL,6) \
			- (1.0/40320.0) * PN * pow(cos(lat),8) * Pt * \
			(-1385.0 + 3111*pow(Pt,2) - 543*pow(Pt,4) + pow(Pt,6)) * pow(PDL,8) ) * Pmo
	    
		m_y = ( PN*cos(lat)*PDL - 1.0/6.0* PN * pow(cos(lat),3) * (-1 + pow(Pt,2) - pow(Pnn,2))*pow(PDL,3) \
			-1.0/120.0*PN*pow(cos(lat),5) * (-5.0+18.0*pow(Pt,2)-pow(Pt,4)-14.0*pow(Pnn,2)+58.0*pow(Pt,2)*pow(Pnn,2))*pow(PDL,5) \
			-1.0/5040.0*PN*pow(cos(lat),7)* (-61.0+479.0*pow(Pt,2)-179.0*pow(Pt,4)+pow(Pt,6))*pow(PDL,7) ) * Pmo
	    
		m_z = height
		
		return m_x, m_y, m_z

	@staticmethod
	def angle2quat (angle):
		angle = float(angle) * pi / 180.0
		return tf.transformations.quaternion_from_euler(0, 0, angle)
		





geoconverter = GeoConvert(7)
currentPosition = [0.0, 0.0, 0.0]
currentOrientation = [0.0, 0.0, 0.0, 0.0]
hasOrientation = False
tfsender = tf.TransformBroadcaster()
devTime = None
posePublisher = None


def nmea_handle_VTG (sentenceComponent):
	pass

def nmea_handle_GGA (sentenceComponent):
	global currentPosition, currentOrientation, devTime
	lat = float (sentenceComponent[2])
	lon = float (sentenceComponent[4])
	height = float (sentenceComponent[9])
	x,y,z = geoconverter.convertNmeaLatLon(lat, lon, height)
	currentPosition = [x, y, z]	

	
def nmea_handle_QQ02C (sentenceComponent):
	global currentPosition, currentOrientation, hasOrientation
	hasOrientation = True
	roll = float(sentenceComponent[4]) * pi / 180.0
	pitch = -float(sentenceComponent[5]) * pi / 180.0
	yaw = -float(sentenceComponent[6]) * pi / 180.0 + pi/2
	#print (roll, pitch, yaw)
# 	currentOrientation = tf.transformations.quaternion_from_euler(roll, pitch, yaw, "syzx")
	hroll = roll * 0.5
	hpitch = pitch * 0.5
	hyaw = yaw * 0.5
	croll = cos(hroll)
	sroll = sin(hroll)
	cpitch = cos(hpitch)
	spitch = sin(hpitch)
	cyaw = cos(hyaw)
	syaw = sin(hyaw)
	currentOrientation = [ sroll * cpitch * cyaw - croll * spitch * syaw, \
						croll * spitch * cyaw + sroll * cpitch * syaw, \
						croll * cpitch * syaw - sroll * spitch * cyaw, \
						croll * cpitch * cyaw + sroll * spitch * syaw ]
	print("Roll={}, Pitch={}, Yaw={}".format(sentenceComponent[4], sentenceComponent[5], sentenceComponent[6]))


def nmea_handle_RMC (sentenceComponent):
	global currentPosition, currentOrientation, hasOrientation
	lat = float (sentenceComponent[3])
	lon = float (sentenceComponent[5])
	# XXX: Need a formula to grab altitude
	height = currentPosition[2]
	x,y,z = geoconverter.convertNmeaLatLon(lat, lon, height)
	currentPosition = [x, y, z]
	if (hasOrientation == False) :
		currentOrientation = GeoConvert.angle2quat(sentenceComponent[8])


def nmea_parse_sentence (sentence):
	cvt = sentence.split (",")
	return cvt


def nmeaCallback (data):
	global currentPosition, currentOrientation, posePublisher
	components = nmea_parse_sentence (data.sentence)
	if (components[0][3:] == "VTG") :
		nmea_handle_VTG (components)
	elif (components[0][3:] == "GGA") :
		nmea_handle_GGA (components)
	elif (components[0][3:] == "RMC") :
		nmea_handle_RMC (components)
	elif (components[0] == "QQ02C") :
		nmea_handle_QQ02C(components)
	#print (currentOrientation)
	tfsender.sendTransform(currentPosition, currentOrientation, data.header.stamp, "gps", "map")
	
	pose = PoseStamped()
	pose.header.frame_id = "map"
	pose.header.stamp = data.header.stamp
	pose.pose.position.x = currentPosition[0]
	pose.pose.position.y = currentPosition[1]
	pose.pose.position.z = currentPosition[2]
	pose.pose.orientation.x = currentOrientation[0]
	pose.pose.orientation.y = currentOrientation[1]
	pose.pose.orientation.z = currentOrientation[2]
	pose.pose.orientation.w = currentOrientation[3]
	posePublisher.publish(pose)
		

if __name__ == "__main__" :
	rospy.init_node ('NmeaListener', anonymous=True)
	rospy.Subscriber ('/nmea_sentence', NmeaSentence, nmeaCallback)
	posePublisher = rospy.Publisher ("gnss_pose", PoseStamped, queue_size=100)
	rospy.spin ()
