/*
    ICE Telescope ROS Package: Baader Planetarium Dome Client
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
#include <string.h>

#define MAX_RETRIES 3

int main(int argc, char **argv)
{
  ros::init(argc, argv, "baader_client");
  if(argc != 2)
  {
    ROS_INFO("Usage: baader_client action. Action: open; close; status");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ice_telescope::baader>("baader_action");
  ice_telescope::baader srv;

  srv.request.baader_action = argv[1];

  for(int i = 0; i < MAX_RETRIES; i++)
  {
    if(client.call(srv))
    {
      if (srv.response.baader_error)
      {
        ROS_ERROR(srv.response.baader_response.c_str());
        if(i < (MAX_RETRIES-1))
          ROS_INFO("Retrying...");
      }
      else
      {
        ROS_INFO(srv.response.baader_response.c_str());
        return 0;
      }
    }
    else
    {
      ROS_ERROR("Failed to call dome service");
      return 1;
    }
  }

  // If we arrive here, the action has not been successful
  return 1;
}
