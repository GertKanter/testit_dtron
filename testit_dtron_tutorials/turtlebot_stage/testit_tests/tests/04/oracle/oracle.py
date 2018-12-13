#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018 Artur Gummel.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Artur Gummel

import roslib
import rospy
import sys
import math
from nav_msgs.msg import Odometry

def getDistance(x1,y1,x2,y2):
	#print x1,y1,x2,y2
	distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
	return distance

def callback(msg1, msg2):
	x1 = msg1.pose.pose.position.x
	y1 = msg1.pose.pose.position.y
	x2 = msg2.pose.pose.position.x
        y2 = msg2.pose.pose.position.y
	return getDistance(x1,y1,x2,y2)

def isInt(value):
  try:
    int(value)
    return True
  except ValueError:
    return False

if __name__ == "__main__":
	rospy.init_node("testit_tb_tutorial")
	rate = rospy.Rate(1) #1 Hz
	numOfRobots = 2

	if len(sys.argv) >= 2:
		if isInt(sys.argv[1]):
			tempNumber = int(sys.argv[1])
			if tempNumber > numOfRobots:
				numOfRobots = tempNumber
		else:
			rospy.loginfo("Number of robots must be of type integer!")

	while not rospy.is_shutdown():
		distanceList = {}

		msg1 = rospy.wait_for_message("/robot_0/base_pose_ground_truth", Odometry)
		for i in range(1, numOfRobots):
			robot_name = "robot_"+str(i)
			msg2 = rospy.wait_for_message("/"+robot_name+"/base_pose_ground_truth", Odometry)
			distanceList[robot_name] = callback(msg1, msg2)
		if len(distanceList) > 0:
			min_key = min(distanceList, key=distanceList.get)
			min_distance = distanceList[min_key]
			if min_distance <= 1.2:
				rospy.loginfo("DONE! %s found robot_0 the distance is %f", min_key, min_distance)
				sys.exit(0)
			else:
				rospy.loginfo("Minimal distance between robot_0 and %s: %f ", min_key, min_distance)
		rate.sleep()

	rospy.spin()
