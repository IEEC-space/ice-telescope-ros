/*
    ICE Telescope ROS Package: Meade LX200GPS Telescope Client
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
#include "ice_telescope/meade.h"
#include <string.h>

void usage_general()
{
  ROS_INFO("Usage: meade_client action [action arguments]. Action: goto; messier; star; deepsky; focus; gps; getobjradec; gettelradec; getdatetime; setdatetime; setlatlon; getlatlon; reconnect");
}

void usage_goto()
{
  ROS_INFO("Usage: meade_client goto ra dec");
  ROS_INFO("Example: meade_client goto 0.73 41.36");
}

void usage_catalog()
{
  ROS_INFO("Usage: meade_client {messier, star, deepsky} objectNum");
  ROS_INFO("Example: meade_client messier 31");
}

void usage_focus()
{
  ROS_INFO("Usage: meade_client focus motion[in/out]");
  ROS_INFO("Example: meade_client focus in");
}

void usage_latlon()
{
  ROS_INFO("Usage: meade_client setlatlon latitude longitude");
  ROS_INFO("Example: meade_client setlatlon 41.385 2.173");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "meade_client");
  if(argc < 2)
  {
    usage_general();
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ice_telescope::meade>("meade_action");
  ice_telescope::meade srv;

  // Action
  srv.request.meade_action = argv[1];

  if(srv.request.meade_action == "goto")
  {
    if(argc != 4)
    {
      usage_goto();
      return 1;
    }

    srv.request.ra = atof(argv[2]);
    srv.request.dec = atof(argv[3]);
  }
  else if(srv.request.meade_action == "messier" || srv.request.meade_action == "star" || srv.request.meade_action == "deepsky")
  {
    if(argc != 3)
    {
      usage_catalog();
      return 1;
    }

    srv.request.object_num = atoi(argv[2]);
  }
  else if(srv.request.meade_action == "focus")
  {
    if(argc != 3)
    {
      usage_focus();
      return 1;
    }

    srv.request.focus_motion = argv[2];
  }
  else if(srv.request.meade_action == "setlatlon")
  {
    if(argc != 4)
    {
      usage_latlon();
      return 1;
    }

    srv.request.lat = atof(argv[2]);
    srv.request.lon = atof(argv[3]);
  }

  if(client.call(srv))
  {
    if (srv.response.meade_error)
    {
      ROS_ERROR(srv.response.meade_response.c_str());
      return 1;
    }
    else
    {
      ROS_INFO(srv.response.meade_response.c_str());
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Failed to call telescope service");
    return 1;
  }
}