/*
    ICE Telescope ROS Package: Baader Planetarium Dome
    Copyright (C) IEEC 2021
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
#include "ice_telescope/BaaderDome.h"

extern "C"
{
  #include "ice_telescope/baader_dome.h"
}


BaaderDome::BaaderDome()
:portFD(-1)
{
  ROS_INFO("Connecting to dome");
  if(dome_connect(&portFD))
  {
    ROS_INFO("Ready to control dome");
  }
  else
  {
    ROS_ERROR("Dome connection failed. Check power and retry");
  }
}

BaaderDome::~BaaderDome()
{
  dome_disconnect(portFD);
}

bool BaaderDome::baader_reconnect(ice_telescope::baader::Response &res)
{
  dome_disconnect(portFD);
  portFD = -1;
  ROS_INFO("Connecting to dome");
  if(dome_connect(&portFD))
  {
    baader_output(res, "Ready to control dome", false);
    return true;
  }

  baader_output(res, "Dome connection failed. Check power and retry", true);
  return false;
}

bool BaaderDome::baader_action(ice_telescope::baader::Request &req, ice_telescope::baader::Response &res)
{
  baader_input(req);

  if(!dome_ack(portFD))
  {
    ROS_ERROR("Dome disconnected");
    if(!baader_reconnect(res))
    {
      return true;
    }
  }

  if(req.baader_action == "open")
  {
    baader_action_open(res);
  }
  else if(req.baader_action == "close")
  {
    baader_action_close(res);
  }
  else if(req.baader_action == "status")
  {
    baader_action_status(res);
  }
  else if(req.baader_action == "reconnect")
  {
    baader_reconnect(res);
  }
  else
  {
    baader_output(res, "Invalid dome action", true);
  }

  return true;
}

void BaaderDome::baader_input(ice_telescope::baader::Request &req)
{
  ROS_INFO("Dome request: %s", req.baader_action.c_str());
}

void BaaderDome::baader_output(ice_telescope::baader::Response &res, string out_str, bool error)
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

void BaaderDome::baader_action_open(ice_telescope::baader::Response &res)
{
  if(dome_control_shutter(portFD, SHUTTER_OPEN)) // Open shutter
  {
    baader_output(res, "Opening shutter... This process can take up to 60 seconds", false);
  }
  else
  {
    baader_output(res, "Error opening shutter", true);
  }
}

void BaaderDome::baader_action_close(ice_telescope::baader::Response &res)
{
  if(dome_control_shutter(portFD, SHUTTER_CLOSE)) // Close shutter
  {
    baader_output(res, "Closing shutter... This process can take up to 60 seconds", false);
  }
  else
  {
    baader_output(res, "Error closing shutter", true);
  }
}

void BaaderDome::baader_action_status(ice_telescope::baader::Response &res)
{
  if(dome_shutter_status(portFD)) // Shutter status
  {
    baader_output(res, dome_get_shutter_status_string(shutterStatus), false);
  }
  else
  {
    baader_output(res, "Error getting shutter status", true);
  }
}
