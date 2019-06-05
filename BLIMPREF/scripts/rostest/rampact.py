#!/usr/bin/env python

import rospy
import actionlib
import sys
import os
import time

from fprime_msgs.msg import RunSeqAction, RunSeqGoal
from mav_msgs.msg import Actuators

if __name__ == '__main__':
   rospy.init_node('rampact')
   pub = rospy.Publisher('flight_actuators_command', Actuators, queue_size=1)
   client = actionlib.SimpleActionClient('ROSSEQ', RunSeqAction)
   client.wait_for_server()

   goal = RunSeqGoal()
   goal.pathToSeq.data = "/seq/arm.bin"
   client.send_goal(goal)
   client.wait_for_result()

   for i in range(0, 200):
      msg = Actuators()
      msg.angular_velocities = [i, i, i, i, i, i, i, i]      
      pub.publish(msg)
      time.sleep(0.01)

   for i in range(200, 0):
      msg = Actuators()
      msg.angular_velocities = [i, i, i, i, i, i, i, i]      
      pub.publish(msg)
      time.sleep(0.01)
   
   goal = RunSeqGoal()
   goal.pathToSeq.data = "/seq/disarm.bin"
   client.send_goal(goal)
   client.wait_for_result()
