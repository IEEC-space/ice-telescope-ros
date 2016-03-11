/*
    ICE Telescope ROS Package: Meade LX200GPS Telescope
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
#include "ice_telescope/MeadeTelescope.h"

extern "C"
{
  #include "ice_telescope/lx200gps.h"
}


MeadeTelescope::MeadeTelescope()
:portFD(-1)
{
  ROS_INFO("Connecting to telescope");
  if(lx200_connect(&portFD))
  {
    ROS_INFO("Ready to control telescope");
  }
  else
  {
    ROS_ERROR("Telescope connection failed. Check power and retry");
  }
}

MeadeTelescope::~MeadeTelescope()
{
  lx200_disconnect(portFD);
}

bool MeadeTelescope::meade_reconnect(ice_telescope::meade::Response &res)
{
  lx200_disconnect(portFD);
  portFD = -1;
  ROS_INFO("Connecting to telescope");
  if(lx200_connect(&portFD))
  {
    meade_output(res, "Ready to control telescope", false);
    return true;
  }

  meade_output(res, "Telescope connection failed. Check power and retry", true);
  return false;
}

bool MeadeTelescope::meade_action(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  if(check_lx200_connection(portFD))
  {
    ROS_ERROR("Telescope disconnected");
    if(!meade_reconnect(res))
    {
      return true;
    }
  }

  if(req.meade_action == "gps")
  {
    meade_input(req, "");
    meade_action_gps(res);
  }
  else if(req.meade_action == "init")
  {
    meade_input(req, "");
    meade_action_init(res);
  }
  else if(req.meade_action == "status")
  {
    meade_input(req, "");
    meade_action_status(res);
  }
  else if(req.meade_action == "park")
  {
    meade_input(req, "");
    meade_action_park(res);
  }
  else if(req.meade_action == "getobjradec")
  {
    meade_input(req, "");
    meade_action_getobjradec(res);
  }
  else if(req.meade_action == "gettelradec")
  {
    meade_input(req, "");
    meade_action_gettelradec(res);
  }
  else if(req.meade_action == "getdatetime")
  {
    meade_input(req, "");
    meade_action_getdatetime(res);
  }
  else if(req.meade_action == "setdatetime")
  {
    meade_input(req, "");
    meade_action_setdatetime(res);
  }
  else if(req.meade_action == "getlatlon")
  {
    meade_input(req, "");
    meade_action_getlatlon(res);
  }
  else if(req.meade_action == "setlatlon")
  {
    meade_input(req, "");
    meade_action_setlatlon(req, res);
  }
  else if(req.meade_action == "focus")
  {
    meade_input(req, req.focus_motion);
    meade_action_focus(req, res);
  }
  else if(req.meade_action == "goto")
  {
    std::stringstream s;
    s << "RA " << req.ra << " - DEC " << req.dec;
    meade_input(req, s.str());
    meade_action_goto(req, res);
  }
  else if(req.meade_action == "messier" || req.meade_action == "star" || req.meade_action == "deepsky")
  {
    std::stringstream s;
    s << req.object_num; // gcc bug where std::to_string() is not found
    meade_input(req, s.str());
    meade_action_catalog(req, res);
  }
  else if(req.meade_action == "reconnect")
  {
    meade_input(req, "");
    meade_reconnect(res);
  }
  else if(req.meade_action == "move")
  {
    std::stringstream s;
    s << "move telescope " << req.motion << " for " << req.time_ms << " ms";
    meade_input(req, s.str());
    meade_action_move(req, res);
  }
  else if(req.meade_action == "sync")
  {
    meade_input(req, "");
    meade_action_sync(req, res);
  }
  else
  {
    meade_output(res, "Invalid telescope action", true);
  }

  return true;
}

void MeadeTelescope::meade_input(ice_telescope::meade::Request &req, string in_str)
{
  if(in_str.empty())
  {
    ROS_INFO("Telescope request: %s", req.meade_action.c_str());
  }
  else
  {
    ROS_INFO("Telescope request: %s %s", req.meade_action.c_str(), in_str.c_str());
  }
}

void MeadeTelescope::meade_output(ice_telescope::meade::Response &res, string out_str, bool error)
{
  res.meade_response = out_str;
  res.meade_error = error;
  if(error)
  {
    ROS_ERROR(res.meade_response.c_str());
  }
  else
  {
    ROS_INFO(res.meade_response.c_str());
  }
}

void MeadeTelescope::meade_action_gps(ice_telescope::meade::Response &res)
{
  gpsRestart(portFD);
  updateGPS_System(portFD);

  meade_output(res, "Updating gps", false);
}

void MeadeTelescope::meade_action_getobjradec(ice_telescope::meade::Response &res)
{
  double obj_ra, obj_dec;

  if(getObjectRA(portFD, &obj_ra) == 0 && getObjectDEC(portFD, &obj_dec) == 0)
  {
    std::stringstream s;
    s << "Target object is set to RA " << obj_ra << " - DEC " << obj_dec;
    meade_output(res, s.str(), false);
  }
  else
  {
    meade_output(res, "Error getting object's RA/DEC", true);
  }
}

void MeadeTelescope::meade_action_gettelradec(ice_telescope::meade::Response &res)
{
  double lx_ra, lx_dec;

  if(getLX200RA(portFD, &lx_ra) == 0 && getLX200DEC(portFD, &lx_dec) == 0)
  {
    std::stringstream s;
    s << "Telescope is at RA " << lx_ra << " - DEC " << lx_dec;
    meade_output(res, s.str(), false);
  }
  else
  {
    meade_output(res, "Error getting telescope's RA/DEC", true);
  }
}

void MeadeTelescope::meade_action_getdatetime(ice_telescope::meade::Response &res)
{
  char ltime[64];
  char date[64];

  if(getCalenderDate(portFD, date) == 0 && getLocTime(portFD, ltime) == 0)
  {
    std::stringstream s;
    s << "Telescope date and time: " << date << " " << ltime;
    meade_output(res, s.str(), false);
  }
  else
  {
    meade_output(res, "Error getting telescope's date and time", true);
  }
}

void MeadeTelescope::meade_action_setdatetime(ice_telescope::meade::Response &res)
{
  struct tm* pTm;
  struct timeval tv;
  struct timezone tz;

  gettimeofday(&tv, &tz);
  pTm = localtime(&(tv.tv_sec));

  if(setCalenderDate(portFD, pTm->tm_mday, pTm->tm_mon + 1, pTm->tm_year + 1900) == 0 && setLocalTime(portFD, pTm->tm_hour, pTm->tm_min, pTm->tm_sec) == 0)
  {
    meade_output(res, "Telescope's date and time set to now", false);
  }
  else
  {
    meade_output(res, "Error setting telescope's date and time", true);
  }
}

void MeadeTelescope::meade_action_getlatlon(ice_telescope::meade::Response &res)
{
  int dd, mm;
  int ddd, mmm;

  if(getSiteLatitude(portFD, &dd, &mm) == 0 && getSiteLongitude(portFD, &ddd, &mmm) == 0)
  {
    std::stringstream s;
    s << "Telescope latitude and longitude: <" << dd << ":" << mm << "> <" << ddd << ":" << mmm << ">";
    meade_output(res, s.str(), false);
  }
  else
  {
    meade_output(res, "Error getting telescope's latitude and longitude", true);
  }
}

void MeadeTelescope::meade_action_setlatlon(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  if(setSiteLatitude(portFD, req.lat) == 0 && setSiteLongitude(portFD, req.lon) == 0)
  {
    std::stringstream s;
    s << "Telescope latitude and longitude set to: " << req.lat << " " << req.lon;
    meade_output(res, s.str(), false);
  }
  else
  {
    meade_output(res, "Error setting telescope's latitude and longitude", true);
  }
}

void MeadeTelescope::meade_action_focus(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  // int motion_type;

  // if(req.focus_motion == "in")
  // {
  //   motion_type = LX200_FOCUSIN;
  // }
  // else if(req.focus_motion == "out")
  // {
  //   motion_type = LX200_FOCUSOUT;
  // }
  // else
  // {
    meade_output(res, "Invalid focus motion. Options are in/out", true);
  // }

  // if(setFocuserMotion(portFD, motion_type) == 0)
  // {
  //   std::stringstream s;
  //   s << "Telescope focusing " << req.focus_motion;
  //   meade_output(res, s.str(), false);
  // }
  // else
  // {
  //   meade_output(res, "Error focusing telescope", true);
  // }
}

void MeadeTelescope::meade_action_goto(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  char* return_msg;

  return_msg = (char*)malloc(64*sizeof(char));
  if(GoTo(portFD, req.ra, req.dec, return_msg))
  {
    meade_output(res, "Telescope slewing to target", false);
  }
  else
  {
    meade_output(res, string(return_msg), true);
  }

  free(return_msg);
}

void MeadeTelescope::meade_action_catalog(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  int catalog = 0;
  int err = 0;

  if(req.meade_action == "messier")
    catalog = LX200_MESSIER_C;
  else if(req.meade_action == "star")
    catalog = LX200_STAR_C;
  else
    catalog = LX200_DEEPSKY_C;

  if(selectCatalogObject(portFD, catalog, req.object_num) == 0)
  {
    if((err = Slew(portFD)) == 0)
    {
      meade_output(res, "Telescope slewing to target", false);
    }
    else
    {
      meade_output(res, string(slewErrorString(err)), true);
    }
  }
  else
  {
    meade_output(res, "Target selection failed", true);
  }
}

void MeadeTelescope::meade_action_park(ice_telescope::meade::Response &res)
{
  if(Park(portFD))
  {
    meade_output(res, "Parking telescope in progress", false);
  }
  else
  {
    meade_output(res, "Parking failed", true);
  }
}

void MeadeTelescope::meade_action_status(ice_telescope::meade::Response &res)
{
  int ret_val = -1;

  if((ret_val = isSlewComplete(portFD)) == 0)
  {
    meade_output(res, "Telescope is IDLE", false);
  }
  else if(ret_val == 1)
  {
    meade_output(res, "Telescope is moving", false);
  }
  else
  {
    meade_output(res, "Error getting telescope status", true);
  }
}

void MeadeTelescope::meade_action_init(ice_telescope::meade::Response &res)
{
  struct tm* pTm;
  struct timeval tv;
  struct timezone tz;

  gettimeofday(&tv, &tz);
  pTm = localtime(&(tv.tv_sec));

  if(remoteInit(portFD, pTm->tm_mday, pTm->tm_mon + 1, pTm->tm_year + 1900, pTm->tm_hour, pTm->tm_min, pTm->tm_sec) == 0)
  {
    meade_output(res, "Telescope is initializing with the current date and time", false);
  } 
  else
  {
    meade_output(res, "Error initializing telescope", true);
  }  
}

void MeadeTelescope::meade_action_move(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  std::stringstream s;

  if(req.motion == "north" || req.motion == "south")
  {
    TelescopeMotionNS dirNS;

    if(req.motion == "north")
      dirNS = MOTION_NORTH;
    else if(req.motion == "south")
      dirNS = MOTION_SOUTH;

    if(MoveNS(portFD, dirNS, MOTION_START))
    {
      ROS_INFO("Moving telescope N/S");
      usleep(req.time_ms * 1000);
      if(MoveNS(portFD, dirNS, MOTION_STOP))
      {
        s << "Telescope moved " << req.motion << " for " << req.time_ms << " ms";
        meade_output(res, s.str(), false);
      }
      else
      {
        meade_output(res, "Error stoping telescope movement N/S", true);
      }
    }
    else
    {
      meade_output(res, "Error starting telescope movement N/S", true);
    }
  }
  else if(req.motion == "east" || req.motion == "west")
  {
    TelescopeMotionWE dirWE;

    if(req.motion == "east")
      dirWE = MOTION_EAST;
    else if(req.motion == "west")
      dirWE = MOTION_WEST;

    if(MoveWE(portFD, dirWE, MOTION_START))
    {
      ROS_INFO("Moving telescope W/E");
      usleep(req.time_ms * 1000);
      if(MoveWE(portFD, dirWE, MOTION_STOP))
      {
        s << "Telescope moved " << req.motion << " for " << req.time_ms << " ms";
        meade_output(res, s.str(), false);
      }
      else
      {
        meade_output(res, "Error stoping telescope movement W/E", true);
      }
    }
    else
    {
      meade_output(res, "Error starting telescope movement W/E", true);
    }
  }
  else
  {
    meade_output(res, "Invalid movement direction. Options are north/south/east/west", true);
  }
}

void MeadeTelescope::meade_action_sync(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  if((req.ra == -1000.0) && (req.dec == -1000.0))
  {
    char* return_msg;
    return_msg = (char*)malloc(64*sizeof(char));

    if(Sync(portFD, return_msg) == 0)
    {
      meade_output(res, "Telescope coordinates synced", false);
    }
    else
    {
      meade_output(res, "Error syncing telescope", true);
    }

    free(return_msg);
  }
  else
  {
    if(SyncRADEC(portFD, req.ra, req.dec))
    {
      meade_output(res, "Telescope coordinates synced", false);
    }
    else
    {
      meade_output(res, "Error syncing telescope", true);
    }
  }
}
