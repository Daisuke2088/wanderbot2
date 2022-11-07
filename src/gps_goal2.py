#!/usr/bin/env python2.7
# coding: UTF-8
from locale import RADIXCHAR
from re import T
from time import time
#from turtle import distance
#from turtle import speed
import rospy
import math
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import pdb ### #pdb.set_trace()#ブレークポイント

goal_lon =8.899999234721275
goal_lat =49.9000510349961
r_g_lon = goal_lon * math.pi / 180
r_g_lat = goal_lat * math.pi / 180
start_longitude = NavSatFix()
Rx = 6378137#長半径[m]
Ry =6356752.3142#短半径[m]
# count=1
# eccentricity = math.sqrt((Rx**2-Ry**2)/Rx**2)

def __init__(self):
  rospy.init_node('gps_goal.py', anonymous=True)
  self.sub = rospy.Subscriber('gps/fix',NavSatFix, self.poseCallback)
  #rospy.Timer(rospy.Duration(1.0), self.timerCallback)
  self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# def poseCallback(self, data):
#   rospy.loginfo("Longitude: %.15f, latitude %.15f" % (data.longitude, data.latitude))
#   longitude = data.longitude
#   latitude = data.latitude
#   global count
#   start_longitude = longitude
#   start_latitude = latitude
#   r_s_lon = start_longitude*math.pi/180#radian start longitude
#   r_s_lat = start_latitude*math.pi/180
        
	#ave_lat = (r_s_lat + r_g_lat)/2
        
        #w = math.sqrt(1-eccentricity**2 * (math.sin(ave_lat))**2) 
        #moving_distance = math.sqrt(((r_s_lat-r_g_lat)*Rx*(1-eccentricity**2)/w**3)**2+((r_s_lon-r_g_lon) * Rx/w * math.cos(ave_lat))**2)
        #rospy.loginfo("distance= %.15f" % moving_distance)
        #target_time = moving_distance/0.5
        #rospy.loginfo("time= %.15f" % target_time)

def DMS_to_decimal_format(lat,long):
  # Check for degrees, minutes, seconds format and convert to decimal
  if ',' in lat:
    degrees, minutes, seconds = lat.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if lat[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    lat = degrees + minutes/60 + seconds/3600
  if ',' in long:
    degrees, minutes, seconds = long.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if long[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    long = degrees + minutes/60 + seconds/3600

  lat = float(lat)
  long = float(long)
  rospy.loginfo('Given GPS goal: lat %s, long %s.' % (lat, long))
  return lat, long

def get_origin_lat_long():
  # Get the lat long coordinates of our map frame's origin which must be publshed on topic /　　local_xy_origin. We use this to calculate our goal within the map frame.
  rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
  origin_pose = rospy.wait_for_message('local_xy_origin', PoseStamped)
  origin_lat = origin_pose.pose.position.y
  origin_long = origin_pose.pose.position.x
  rospy.loginfo('Received origin: lat %s, long %s.' % (origin_lat, origin_long))
 
  return origin_lat, origin_long

def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
  hypotenuse = distance = g['s12'] # access distance
  rospy.loginfo("The distance from the origin to the goal is {:.3f} m.".format(distance))
  azimuth = g['azi1']
  rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse
  rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

  return x, y
class GpsGoal():
  def __init__(self):
	if count==1:
	    
	    speed=0.5#[m/s]
	    stop=0
	    t = Twist()
	    t_s = Twist()
	    t.linear.x=speed
	    t_s.linear.x=stop
	    start_time = rospy.Time.now() + rospy.Duration(target_time)

	    while rospy.Time.now() < start_time:
		self.pub.publish(t)
		
	    else:
		self.pub.publish(t_s)
	    count=count+1

	else:         
	    if moving_distance<0.5:
		return 
	    else:
		while rospy.Time.now() < start_time:
		    self.pub.publish(t)
		
		else:
		    self.pub.publish(t_s)          


# def ros_main():
#   gpsGoal = GpsGoal();
#   rospy.spin()

if __name__ == '__main__':
  try:
    gpsGoal = GpsGoal()
    rospy.spin()
  except rospy.ROSInterruptException: pass