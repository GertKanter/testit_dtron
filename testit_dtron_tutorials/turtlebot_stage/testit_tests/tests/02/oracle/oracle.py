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
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped

def resultCallback(data):
	goalStatus = data.status.text
	rospy.loginfo(goalStatus)
	if goalStatus == "Failed to find a valid plan. Even after executing recovery behaviors.":
		return False
	else:
		return True

def goalCallback(data):
	x = float(data.pose.position.x)
	y = float(data.pose.position.y)
	rospy.loginfo('Current goal X: %f Y: %f', x, y)
	msgResult = rospy.wait_for_message("/move_base/result", MoveBaseActionResult)
	reachedLocation = resultCallback(msgResult)
	if not reachedLocation:
		rospy.loginfo("Did not reach WayPoint x: %f y: %f", x, y)
		return 1 #fail
	else:
		return 3 #continue

if __name__ == "__main__":
	rospy.init_node("testit_tb_tutorial")
	rate = rospy.Rate(2)
	counter = 10
	if len(sys.argv) == 2:
		counter = int(sys.argv[1])
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/move_base/current_goal", PoseStamped)
		result = goalCallback(msg)
		if result == 3:
			counter -= 1
			if counter <= 0:
				sys.exit(0) #success
		elif result == 1:
			sys.exit(1) #fail
		rate.sleep()