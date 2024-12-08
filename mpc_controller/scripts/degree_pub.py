#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from math import radians
from tf.transformations import quaternion_from_euler
from numpy import unwrap
import math
from tf.transformations import euler_from_quaternion
import sys, signal


def signal_handler(sig, frame):
    print("\nExecution interrupted by user.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class degree_publisher:
  def __init__(self):

    rospy.init_node('degree_publisher')
    self.sub = rospy.Subscriber("/qualisys/olympus/odom", Odometry, self.poseCallback)
    self.pub= rospy.Publisher('/olympus/z_axis_angle', Vector3Stamped, queue_size=10)
    self.rate = rospy.Rate(100)  # 10 Hz

    self.pub_msg = Vector3Stamped()
    self.pub_msg.vector.x = 0
    self.pub_msg.vector.y = 0
    self.pub_msg.vector.z = 0

    self.current_angle = 0


  def update(self):
     while not rospy.is_shutdown():
      self.rate.sleep()  

  def poseCallback(self,msg):
    self.pub_msg.vector.z =  self.get_angle( msg.pose.pose.orientation )
    self.pub_msg.header.stamp = rospy.Time.now()
    self.pub.publish(self.pub_msg)  
     
  def get_angle(self, orientation):
    roll, pitch, yaw = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w]) #returns radians
    unwraped_yaw = unwrap([self.current_angle, yaw])
    self.current_angle = unwraped_yaw[1]
    #In qualisys -> everything is in z direction
    return math.degrees(self.current_angle)

    
if __name__ == '__main__':
  try:
      plotter_obj= degree_publisher()
      plotter_obj.update()
  except rospy.ROSInterruptException:
      pass