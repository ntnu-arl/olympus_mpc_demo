#!/usr/bin/env python

import rospy
import sys, signal
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

def signal_handler(sig, frame):
    print("\nExecution interrupted by user.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class reference_publisher:
  def __init__(self):
    
    # User Input : 
    self.moving_ref_mode = False
    self.Ref  = [ -90]
    self.DT   = [ 0]
    self.lamda = 2

    # self.Ref  = [ 45, -45, -120]
    # self.DT   = [ 10,  15,   15] 


    # Node Init: 
    rospy.init_node('reference_publisher')
    self.pub  = rospy.Publisher('/olympus/desired_angle', Float32MultiArray, queue_size=10)

    if self.moving_ref_mode:
      self.current_z_angle = 0
      try:
        Vectormsg = rospy.wait_for_message('/olympus/z_axis_angle', Vector3Stamped,5)
        self.current_z_angle = Vectormsg.vector.z
      except rospy.exceptions.ROSException as e:
        rospy.loginfo("[reference publisher]: Did not receive current angle, assuming 0")
           
    self.rate = rospy.Rate(2)  # 10 Hz
    self.time_prev = rospy.get_time()
    self.index = 0
    self.N = self.Ref.__len__()

    self.pub_msg = Float32MultiArray()
    self.pub_msg.data = [self.Ref[0]]

  def update(self):
     while not rospy.is_shutdown() :

      if self.moving_ref_mode:
         self.moving_ref()
      else:
        self.update_reference()

      self.pub.publish(self.pub_msg)
      self.rate.sleep()  

  def update_reference(self):
     current_time = rospy.get_time()
     if self.index < self.N-1:
        if current_time - self.time_prev >= self.DT[self.index]:
          self.time_prev    = current_time
          self.index        = self.index +1 
          self.pub_msg.data = [self.Ref[self.index]]

  def moving_ref(self):
    current_time = rospy.get_time()
    self.pub_msg.data = [self.lamda * (current_time -self.time_prev) + self.current_z_angle]
    
if __name__ == '__main__':
  try:
      plotter_obj= reference_publisher()
      plotter_obj.update()
  except rospy.ROSInterruptException:
      pass