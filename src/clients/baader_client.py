#!/usr/bin/env python

#    ICE Telescope ROS Package: Baader Planetarium Dome Client
#    Copyright (C) 2015 Biel Artigues Aguilo
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import rospy
from ice_telescope.srv import *

MAX_RETRIES = 3

def usage():
    return "Usage: baader_client action. Action: open; close; status"

def baader_action_client(action):
    rospy.wait_for_service('baader_action')
    try:
        srv = rospy.ServiceProxy('baader_action', baader)
        resp = srv(action)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('baader_client')
    if len(sys.argv) == 2:
        action = str(sys.argv[1])
    else:
        rospy.loginfo("%s", usage())
        sys.exit(1)
    
    for i in range(0, MAX_RETRIES):
        resp = baader_action_client(action)
        if resp.baader_error:
            rospy.logerr("%s", resp.baader_response)
            if i < (MAX_RETRIES-1):
                rospy.loginfo("Retrying...")
        else:
            rospy.loginfo("%s", resp.baader_response)
            sys.exit(0)

    # If we arrive here, the action has not been successful
    sys.exit(1)
