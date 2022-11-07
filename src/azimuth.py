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
from geographiclib.geodesic import Geodesic
import pdb ### #pdb.set_trace()#ブレークポイント

#goal_lon =8.899999234721275#直進のみ
goal_lon = 8.899966497139753#旋回直進用
#goal_lat = 49.9000510349961#直進のみ
goal_lat = 49.90001544189581#旋回直進用
r_g_lon = goal_lon * math.pi / 180
r_g_lat = goal_lat * math.pi / 180
start_longitude = NavSatFix()
Rx = 6378137#長半径[m]
Ry =6356752.3142#短半径[m]
count=1
eccentricity = math.sqrt((Rx**2-Ry**2)/Rx**2)
class turtlebot_goal:
    def __init__(self):
        rospy.init_node('azimuth.py', anonymous=True)
        self.sub = rospy.Subscriber('gps/fix',NavSatFix, self.poseCallback)
        #rospy.Timer(rospy.Duration(1.0), self.timerCallback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    def poseCallback(self, data):
        rospy.loginfo("Longitude: %.15f, latitude %.15f" % (data.longitude, data.latitude))
        longitude = data.longitude
        latitude = data.latitude
        global count
        start_longitude = longitude
        start_latitude = latitude
        r_s_lon = start_longitude*math.pi/180#radian start longitude
        r_s_lat = start_latitude*math.pi/180
        ave_lat = (r_s_lat + r_g_lat)/2
        
        w = math.sqrt(1-eccentricity**2 * (math.sin(ave_lat))**2) 
        moving_distance = math.sqrt(((r_s_lat-r_g_lat)*Rx*(1-eccentricity**2)/w**3)**2+((r_s_lon-r_g_lon) * Rx/w * math.cos(ave_lat))**2)
        rospy.loginfo("distance= %.15f" % moving_distance)

        geod = Geodesic.WGS84  # define the WGS84 ellipsoid #追加
        g = geod.Inverse(r_s_lat, r_s_lon, r_g_lat, r_g_lon) # Compute several geodesic calculations between two GPS points  #追加
        azimuth = g['azi1'] #追加
        rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth)) #追加
        target_time = moving_distance/0.5
        rospy.loginfo("linear_move_time= %.15f" % target_time)
        
    #def pub_z(self,azimuth):
        speed_azi = 1#[deg/s]
        stop = 0#[deg/s]
        target_time_azi = math.fabs( azimuth / speed_azi )#[s]
        rospy.loginfo("turning_time= %.15f" % target_time_azi)
        t = Twist()
        t_s = Twist()
        t.linear.z = speed_azi
        t_s.linear.z = stop
        start_time_azi = rospy.Time.now() + rospy.Duration(target_time_azi)
        b = rospy.Time.now()
        # start_time_azi = time.time()
        # end_time_azi = time.time()
        # rate = rospy.Rate(30)

        # while end_time_azi - start_time_azi <= target_time_azi:
        #     self.pub.publish(t)
        #     end_time_azi = time.time()
        #     rate.sleep()
        while rospy.Time.now() < start_time_azi:
                self.pub.publish(t)
                
        else:
            self.pub.publish(t_s)

        # while rospy.Time.now() < start_time:
        #     self.pub.publish(t)
        # else:
        #     self.pub.publish(t_s)
        

    # def pub_x(self):
    #     speed=0.5#[m/s]
    #     stop=0
    #     t = Twist()
    #     t_s = Twist()
    #     t.linear.x=speed
    #     t_s.linear.x=stop
    #     start_time = rospy.Time.now() + rospy.Duration(target_time)

    #     while rospy.Time.now() < start_time:
    #         self.pub.publish(t)
            
    #     else:
    #         self.pub.publish(t_s)
    #     # count=count+1

    # else:         
    #     if moving_distance<0.5:
    #         return 
    #     else:
    #         while rospy.Time.now() < start_time:
    #             self.pub.publish(t)
            
    #         else:
    #             self.pub.publish(t_s)          

if __name__ == '__main__':
    try:
        ts = turtlebot_goal()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    # try:
    # ts = turtlebot_goal()
    # rospy.spin()
    # azimuth = ts.poseCallback()
    # ts.pub_z()
    # ts.pub_x()


    # except rospy.ROSInterruptException: pass

    


