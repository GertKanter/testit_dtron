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
import actionlib
import rospy
import testit_oracles.testit_gazebo
import sys
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped

def resultCallback(data):
	goalStatus = data.status.text
	rospy.loginfo(goalStatus)
	if goalStatus == "Goal reached.":
		return True
	else:
		return False

def goalCallback(data, waypointX, waypointY):
	x = float(data.pose.position.x)
	y = float(data.pose.position.y)
	rospy.loginfo('Current goal X: %f Y: %f', x, y)
	if waypointX - 0.1 <= x <= waypointX + 0.1 and waypointY - 0.1 <= y <= waypointY + 0.1:
		msgResult = rospy.wait_for_message("/move_base/result", MoveBaseActionResult)
		reachedLocation = resultCallback(msgResult)
		if reachedLocation:
			rospy.loginfo("WayPoint x: %f y: %f reached successfully", x, y)
			return 0 #success
		else:
			rospy.loginfo("Did not reach WayPoint x: %f y: %f", x, y)
			return 1 #fail
	else:
		rospy.loginfo("Wait for a next goal")
		return 3 #continue

if __name__ == "__main__":
	rospy.init_node("testit_tb_tutorial")
	rate = rospy.Rate(2)
	waypointX = float(sys.argv[1])
	waypointY = float(sys.argv[2])
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/move_base/current_goal", PoseStamped)
		result = goalCallback(msg, waypointX, waypointY)
		rospy.loginfo("GOT RETURN STATEMENT %d", result)
		if result == 0:
			sys.exit(0)
		elif result == 1:
			sys.exit(1)
		rate.sleep()