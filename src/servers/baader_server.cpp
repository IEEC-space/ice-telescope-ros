/*
    ICE Telescope ROS Package: Baader Planetarium Dome Server
    Copyright (C) 2015 Biel Artigues Aguilo

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
#include "ice_telescope/baader.h"
#include <string>
using namespace std;

extern "C"
{
  #include "ice_telescope/baader_dome.h"
}

void baader_output(ice_telescope::baader::Response &res, string out_str, bool error)
{
  res.baader_response = out_str;
  res.baader_error = error;
  if(error)
  {
    ROS_ERROR(res.baader_response.c_str());
  }
  else
  {
    ROS_INFO(res.baader_response.c_str());
  }
}

bool baader_action(ice_telescope::baader::Request  &req,
                  ice_telescope::baader::Response &res)
{
  if(dome_connect())
  {
    //ROS_INFO("Dome connected");
    ROS_INFO("Dome request: %s", req.baader_action.c_str());

    if(req.baader_action == "open")
    {
      if(dome_control_shutter(SHUTTER_OPEN)) // Open shutter
      {
        baader_output(res, "Opening shutter... This process can take up to 60 seconds", false);
      }
      else
      {
        baader_output(res, "Error opening shutter", true);
      }
    }
    else if(req.baader_action == "close")
    {
      if(dome_control_shutter(SHUTTER_CLOSE)) // Close shutter
      {
        baader_output(res, "Closing shutter... This process can take up to 60 seconds", false);
      }
      else
      {
        baader_output(res, "Error closing shutter", true);
      }
    }
    else if(req.baader_action == "status")
    {
      if(dome_shutter_status()) // Shutter status
      {
        baader_output(res, dome_get_shutter_status_string(shutterStatus), false);
      }
      else
      {
        baader_output(res, "Error getting shutter status", true);
      }
    }
    else
    {
      baader_output(res, "Invalid dome action", true);
    }

    dome_disconnect();
    //ROS_INFO("Dome disconnected");
  }
  else
  {
    baader_output(res, "Dome connection failed", true);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baader_server");
  ros::NodeHandle n;

  // Dome service
  ros::ServiceServer baaderService = n.advertiseService("baader_action", baader_action);
  ROS_INFO("Ready to control dome");

  ros::spin();

  return 0;
}
