/*
    ICE Telescope ROS Package: Vaisala Weather Station
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
#include "ice_telescope/VaisalaWS.h"

extern "C"
{
  #include "ice_telescope/vaisala_ws.h"
}

VaisalaWS::VaisalaWS()
:portFD(-1)
{
  ROS_INFO("Connecting to weather station");
  if(ws_connect(&portFD))
  {
    ROS_INFO("Ready to control weather station");
  }
  else
  {
    ROS_ERROR("Weather station connection failed. Check power and retry");
  }
}

VaisalaWS::~VaisalaWS()
{
  ws_disconnect(portFD);
}

bool VaisalaWS::vaisala_reconnect(ice_telescope::vaisala::Response &res)
{
  ws_disconnect(portFD);
  portFD = -1;
  ROS_INFO("Connecting to weather station");
  if(ws_connect(&portFD))
  {
    vaisala_output(res, "Ready to control weather station", false);
    return true;
  }

  vaisala_output(res, "Weather station connection failed. Check power and retry", true);
  return false;
}

bool VaisalaWS::vaisala_action(ice_telescope::vaisala::Request &req, ice_telescope::vaisala::Response &res)
{
  vaisala_input(req);

  if(!ws_ack(portFD))
  {
    ROS_ERROR("Weather station disconnected");
    if(!vaisala_reconnect(res))
    {
      return true;
    }
  }

  if(req.vaisala_action == "getinfo")
  {
    vaisala_action_getinfo(res);
  }
  else
  {
    vaisala_output(res, "Invalid weather station action", true);
  }

  return true;
}

void VaisalaWS::vaisala_input(ice_telescope::vaisala::Request &req)
{
  ROS_INFO("Weather station request: %s", req.vaisala_action.c_str());
}

void VaisalaWS::vaisala_output(ice_telescope::vaisala::Response &res, string out_str, bool error)
{
  res.vaisala_response = out_str;
  res.vaisala_error = error;
  if(error)
  {
    ROS_ERROR(res.vaisala_response.c_str());
  }
  else
  {
    ROS_INFO(res.vaisala_response.c_str());
  }
}

void VaisalaWS::vaisala_action_getinfo(ice_telescope::vaisala::Response &res)
{
  char info[64];

  if(ws_getinfo(portFD, info) == 0)
  {
    std::stringstream s;
    s << "Weather info: " << info;
    vaisala_output(res, s.str(), false);
  }
  else
  {
    vaisala_output(res, "Error getting info from weather station", true);
  }
}

