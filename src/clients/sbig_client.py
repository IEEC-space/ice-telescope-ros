#!/usr/bin/env python

#    ICE Telescope ROS Package: SBIG CCD Client
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

def usage_general():
	rospy.loginfo("Usage: sbig_client action [action arguments]. Action: capture; settemp; gettemp; getcapstatus; reconnect")

def usage_capture():
	rospy.loginfo("Usage capture: sbig_client capture filePath fileType imgCount imgType expTime readoutMode top left width height fastReadout dualReadoutChannel")
	rospy.loginfo("Example: sbig_client capture /observations/img/raw/ FITS 1 LF 0.01 1x1 0 0 0 0 1 1")
	rospy.loginfo("Example: sbig_client capture /observations/img/raw/ SBIG 1 LF 0.01 1x1 0 0 0 0 0 0")

def usage_settemp():
	rospy.loginfo("Usage settemp: sbig_client settemp enable temperature")
	rospy.loginfo("Example: sbig_client settemp 1 10.0")

def sbig_action_client(sbig_action, file_path, fits_file, img_count, lf_img, exp_time, readout_mode, top, left, width, height, fast_readout, dual_readout_channel, temperature, temp_enable):
    rospy.wait_for_service('sbig_action')
    try:
        srv = rospy.ServiceProxy('sbig_action', sbig)
        if sbig_action == "capture":
        	rospy.loginfo("Starting CCD exposure")
        resp = srv(sbig_action, file_path, fits_file, img_count, lf_img, exp_time, readout_mode, top, left, width, height, fast_readout, dual_readout_channel, temperature, temp_enable)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":
	rospy.init_node('sbig_client')
	if len(sys.argv) < 2:
		usage_general()
		sys.exit(1)

	sbig_action = str(sys.argv[1])

	if sbig_action == "capture":
		if len(sys.argv) != 14:
			usage_capture()
			sys.exit(1)

		file_path = str(sys.argv[2])
		if not file_path.endswith("/"):
			file_path = file_path + '/'

		fits_file = False
		if str(sys.argv[3]) == "FITS":
			fits_file = True

		img_count = int(sys.argv[4])

		lf_img = False
		if str(sys.argv[5]) == "LF":
			lf_img = True

		exp_time = float(sys.argv[6])

		readout_mode = 0
		if str(sys.argv[7]) == "2x2":
			readout_mode = 1
		elif str(sys.argv[7]) == "3x3":
			readout_mode = 2

		top = int(sys.argv[8])
		left = int(sys.argv[9])
		width = int(sys.argv[10])
		height = int(sys.argv[11])

		fast_readout = False
		if str(sys.argv[12]) == "1":
			fast_readout = True

		dual_readout_channel = False
		if str(sys.argv[13]) == "1":
			dual_readout_channel = True

		temperature = None
		temp_enable = None

	elif sbig_action == "settemp":
		if len(sys.argv) != 4:
			usage_settemp()
			sys.exit(1)

		file_path = None
		fits_file = None
		img_count = None
		lf_img = None
		exp_time = None
		readout_mode = None
		top = None
		left = None
		width = None
		height = None
		fast_readout = None
		dual_readout_channel = None
		temperature = float(sys.argv[3])
		temp_enable = False
		if str(sys.argv[2]) == "1":
			temp_enable = True

	else:
		file_path = None
		fits_file = None
		img_count = None
		lf_img = None
		exp_time = None
		readout_mode = None
		top = None
		left = None
		width = None
		height = None
		fast_readout = None
		dual_readout_channel = None
		temperature = None
		temp_enable = None


	resp = sbig_action_client(sbig_action, file_path, fits_file, img_count, lf_img, exp_time, readout_mode, top, left, width, height, fast_readout, dual_readout_channel, temperature, temp_enable)
	if resp.sbig_error:
		rospy.logerr("%s", resp.sbig_response)
		sys.exit(1)
	else:
		rospy.loginfo("%s", resp.sbig_response)
		sys.exit(0)
