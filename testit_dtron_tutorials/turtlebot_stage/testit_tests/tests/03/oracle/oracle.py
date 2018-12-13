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
import sys
from topological_navigation.msg import *


def goalCallback(data):
	target = data.goal.target
	goalId = data.goal_id.id
	rospy.loginfo('Current goal %s', target)
	msgResult = rospy.wait_for_message("/topological_navigation/result", GotoNodeActionResult)
	reachedLocation = msgResult.result.success
	reachedTargetId = msgResult.status.goal_id.id
	if reachedLocation and goalId == reachedTargetId:
		rospy.loginfo("%s reached successfully", target)
		return 0
	elif not reachedLocation:
		rospy.loginfo("Did not reach %s", target)
		return 1
	else:
		rospy.loginfo("Wait for a next goal")
	return 3

if __name__ == "__main__":
	rospy.init_node("testit_tb_tutorial")
	reachWayPoint = sys.argv[1]
	rospy.loginfo("I need to check %s", reachWayPoint)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/topological_navigation/goal",GotoNodeActionGoal)
		if reachWayPoint == msg.goal.target:
			result = goalCallback(msg)
			rospy.loginfo("GOT RETURN STATEMENT %d", result)
			if result == 0:
				sys.exit(0) #success
			elif result == 1:
				sys.exit(1) #fail
		rate.sleep()