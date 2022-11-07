#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import radians
from sensor_msgs.msg import NavSatFix

lat = 0.0
lon = 0.0


class sub():

  def __init__(self):
     rospy.Subscriber("/gps/fix", NavSatFix, self.call_2)
  def call_2(self, gps):
      
      global lat
      global lon

      lat = gps.latitude
      lon = gps.longitude
      print(lat)
      print(lon)

#def shutdown(self):
 #   rospy.loginfo("Stop Turtlebot")
  #  cmd_vel.publish(Twist())
   # rospy.sleep(1)

#cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) #<1>
#gps_fix_sub = rospy.Subscriber('gps/fix', NavSatFix, queue_size=1)
#rospy.init_node('red_light_green_light')

#def callback(data):
 #   rospy.loginfo("%f,%f" % (data.longitude, data.latitude))#loginfo
  #  longitude = data.longitude
   # latitude = data.latitude
       # goal_longitude = NavSatFix()
#49.90178
   # goal_latitude = NavSatFix()
#8.8952

  #  moving_distance = 6371000.685 * (math.sqrt((data.latitude - goal_latitude)**2+( data.longitude - goal_longitude)**2))
 #   moving_time = rospy.
  #  moving_time = moving_distance / 0.5

#red_light_twist = Twist() #<2>
#green_light_twist = Twist()
#green_light_twist.linear.x = 0.5 #<3>

#driving_forward = False
#light_change_time = rospy.Time.now()
#rate = rospy.Rate(10)

#while not rospy.is_shutdown():
 # if driving_forward:
  #  cmd_vel_pub.publish(green_light_twist) #<4>
  #else:
   # cmd_vel_pub.publish(red_light_twist)

#    moving_distance = 6371000.685 * (math.sqrt((latitude - goal_latitude)**2 + (goal_longitude)**2))
 #   moving_time = moving_distance / 0.5

#def callback(moving_time):
  # BEGIN PART_1
 # if rospy.Time.now() > light_change_time: #<5>
  #  driving_forward = not driving_forward
   # light_change_time = rospy.Time.now() + rospy.Duration(moving_time)
  # END PART_1
  rate.sleep() #<6>
# END ALL
