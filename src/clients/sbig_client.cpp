/*
    ICE Telescope ROS Package: SBIG CCD Client
    Copyright (C) IEEC 2020
    Author: Biel Artigues Aguilo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ros/ros.h"
#include "ice_telescope/sbig.h"
#include <string.h>

void usage_general()
{
  ROS_INFO("Usage: sbig_client action [action arguments]. Action: capture; settemp; gettemp; getcapstatus; reconnect");
}

void usage_capture()
{
  ROS_INFO("Usage capture: sbig_client capture filePath fileType imgCount imgType expTime readoutMode top left width height fastReadout dualReadoutChannel");
  ROS_INFO("Example: sbig_client capture /observations/img/raw/ FITS 1 LF 0.01 1x1 0 0 0 0 1 1");
  ROS_INFO("Example: sbig_client capture /observations/img/raw/ SBIG 1 LF 0.01 1x1 0 0 0 0 0 0");
}

void usage_settemp()
{
  ROS_INFO("Usage settemp: sbig_client settemp enable temperature");
  ROS_INFO("Example: sbig_client settemp 1 10.0");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbig_client");
  if(argc < 2)
  {
    usage_general();
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ice_telescope::sbig>("sbig_action");
  ice_telescope::sbig srv;

  // Action
  srv.request.sbig_action = argv[1];

  if(srv.request.sbig_action == "capture")
  {
    if(argc != 14)
    {
      usage_capture();
      return 1;
    }

    // File path
    srv.request.file_path = argv[2];
    int len = srv.request.file_path.length();
    if(srv.request.file_path[len - 1] != '/')
    {
      srv.request.file_path.append("/");
    }

    // File type
    srv.request.fits_file = false;
    if(strcmp(argv[3], "FITS") == 0)
    {
      srv.request.fits_file = true;
    }

    // Image count
    srv.request.img_count = atoi(argv[4]);

    // Image type. Suppose DF
    srv.request.lf_img = false;
    if(strcmp(argv[5], "LF") == 0)
    {
      srv.request.lf_img = true;
    }

    // Exposure time
    srv.request.exp_time = atof(argv[6]);

    // Readout mode. Suppose 1x1
    srv.request.readout_mode = 0;
    if(strcmp(argv[7], "2x2") == 0)
    {
      srv.request.readout_mode = 1;
    }
    else if(strcmp(argv[7], "3x3") == 0)
    {
      srv.request.readout_mode = 2;
    }

    // Image size: if all params, ie. top, left, width and height are zero,
    // the full size of the CCD image is used.
    srv.request.top = atoi(argv[8]);
    srv.request.left = atoi(argv[9]);
    srv.request.width = atoi(argv[10]);
    srv.request.height = atoi(argv[11]);

    // Fast readout
    srv.request.fast_readout = false;
    if(strcmp(argv[12], "1") == 0)
    {
      srv.request.fast_readout = true;
    }

    // Dual channel mode
    srv.request.dual_readout_channel = false;
    if(strcmp(argv[13], "1") == 0)
    {
      srv.request.dual_readout_channel = true;
    }

    ROS_INFO("Starting CCD exposure");
  }
  else if(srv.request.sbig_action == "settemp")
  {
    if(argc != 4)
    {
      usage_settemp();
      return 1;
    }

    srv.request.temperature = atof(argv[3]);
    srv.request.temp_enable = false;
    if(strcmp(argv[2], "1") == 0)
    {
      srv.request.temp_enable = true;
    }
  }

  if(client.call(srv))
  {
    if(srv.response.sbig_error)
    {
      ROS_ERROR(srv.response.sbig_response.c_str());
      return 1;
    }
    else
    {
      ROS_INFO(srv.response.sbig_response.c_str());
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Failed to call ccd service");
    return 1;
  }
}
