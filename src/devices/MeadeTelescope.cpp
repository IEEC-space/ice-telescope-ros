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
{

}

MeadeTelescope::~MeadeTelescope()
{

}

bool MeadeTelescope::meade_action(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
{
  int portFD = -1;

  ROS_INFO("Connecting to telescope");
  if(lx200_connect(&portFD))
  {
    if(req.meade_action == "gps")
    {
      meade_input(req, NULL);
      meade_action_gps(portFD, res);
    }
    else if(req.meade_action == "getobjradec")
    {
      meade_input(req, NULL);
      meade_action_getobjradec(portFD, res);
    }
    else if(req.meade_action == "gettelradec")
    {
      meade_input(req, NULL);
      meade_action_gettelradec(portFD, res);
    }
    else if(req.meade_action == "getdatetime")
    {
      meade_input(req, NULL);
      meade_action_getdatetime(portFD, res);
    }
    else if(req.meade_action == "setdatetime")
    {
      meade_input(req, NULL);
      meade_action_setdatetime(portFD, res);
    }
    else if(req.meade_action == "getlatlon")
    {
      meade_input(req, NULL);
      meade_action_getlatlon(portFD, res);
    }
    else if(req.meade_action == "setlatlon")
    {
      meade_input(req, NULL);
      meade_action_setlatlon(portFD, req, res);
    }
    else if(req.meade_action == "focus")
    {
      meade_input(req, req.focus_motion);
      meade_action_focus(portFD, req, res);
    }
    else if(req.meade_action == "goto")
    {
      std::stringstream s;
      s << "RA " << req.ra << " - DEC " << req.dec;
      meade_input(req, s.str());
      meade_action_goto(portFD, req, res);
    }
    else if(req.meade_action == "messier" || req.meade_action == "star" || req.meade_action == "deepsky")
    {
      std::stringstream s;
      s << req.object_num; // gcc bug where std::to_string() is not found
      meade_input(req, s.str());
      meade_action_catalog(portFD, req, res);
    }
    else
    {
      meade_output(res, "Invalid telescope action", true);
    }

    lx200_disconnect(portFD);
  }
  else
  {
    meade_output(res, "Telescope connection failed", true);
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

void MeadeTelescope::meade_action_gps(int portFD, ice_telescope::meade::Response &res)
{
  gpsRestart(portFD);
  updateGPS_System(portFD);

  meade_output(res, "Updating gps", false);
}

void MeadeTelescope::meade_action_getobjradec(int portFD, ice_telescope::meade::Response &res)
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

void MeadeTelescope::meade_action_gettelradec(int portFD, ice_telescope::meade::Response &res)
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

void MeadeTelescope::meade_action_getdatetime(int portFD, ice_telescope::meade::Response &res)
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

void MeadeTelescope::meade_action_setdatetime(int portFD, ice_telescope::meade::Response &res)
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

void MeadeTelescope::meade_action_getlatlon(int portFD, ice_telescope::meade::Response &res)
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

void MeadeTelescope::meade_action_setlatlon(int portFD, ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
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

void MeadeTelescope::meade_action_focus(int portFD, ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
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

void MeadeTelescope::meade_action_goto(int portFD, ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
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
}

void MeadeTelescope::meade_action_catalog(int portFD, ice_telescope::meade::Request &req, ice_telescope::meade::Response &res)
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