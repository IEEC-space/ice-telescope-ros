#!/usr/bin/env python

#    ICE Telescope ROS Package: Meade LX200GPS Telescope Client
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
from std_msgs.msg import String
from ice_telescope.srv import *

def usage_general():
    rospy.loginfo("Usage: meade_client action [action arguments]. Action: goto; messier; star; deepsky; focus; gps; getobjradec; gettelradec; getdatetime; setdatetime; setlatlon; getlatlon")

def usage_goto():
    rospy.loginfo("Usage: meade_client goto ra dec")
    rospy.loginfo("Example: meade_client goto 0.73 41.36")

def usage_catalog():
    rospy.loginfo("Usage: meade_client {messier, star, deepsky} objectNum")
    rospy.loginfo("Example: meade_client messier 31")

def usage_focus():
    rospy.loginfo("Usage: meade_client focus motion[in/out]")
    rospy.loginfo("Example: meade_client focus in")

def usage_latlon():
    rospy.loginfo("Usage: meade_client setlatlon latitude longitude")
    rospy.loginfo("Example: meade_client setlatlon 41.385 2.173")

def retryCallback(msg):
    rospy.logerr(msg.data)

def meade_action_client(action, ra, dec, object_num, focus_motion, lat, lon):
    rospy.wait_for_service('meade_action')
    try:
        srv = rospy.ServiceProxy('meade_action', meade)
        resp = srv(action, ra, dec, object_num, focus_motion, lat, lon)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('meade_client')
    rospy.Subscriber("retry_meade", String, retryCallback)
    if len(sys.argv) < 2:
        usage_general()
        sys.exit(1)

    action = str(sys.argv[1])

    if action == "goto":
        if len(sys.argv) != 4:
            usage_goto()
            sys.exit(1)

        ra = float(sys.argv[2])
        dec = float(sys.argv[3])
        object_num = None
        focus_motion = None
        lat = None
        lon = None

    elif action == "messier" or action == "star" or action == "deepsky":
        if len(sys.argv) != 3:
            usage_catalog()
            sys.exit(1)

        ra = None
        dec = None
        object_num = int(sys.argv[2])
        focus_motion = None
        lat = None
        lon = None

    elif action == "focus":
        if len(sys.argv) != 3:
            usage_focus()
            sys.exit(1)

        ra = None
        dec = None
        object_num = None
        focus_motion = str(sys.argv[2])
        lat = None
        lon = None

    elif action == "setlatlon":
        if len(sys.argv) != 4:
            usage_latlon()
            sys.exit(1)

        ra = None
        dec = None
        object_num = None
        focus_motion = None
        lat = float(sys.argv[2])
        lon = float(sys.argv[3])

    else:
        ra = None
        dec = None
        object_num = None
        focus_motion = None
        lat = None
        lon = None

    
    resp = meade_action_client(action, ra, dec, object_num, focus_motion, lat, lon)
    if resp.meade_error:
        rospy.logerr("%s", resp.meade_response)
        sys.exit(1)
    else:
        rospy.loginfo("%s", resp.meade_response)
        sys.exit(0)
