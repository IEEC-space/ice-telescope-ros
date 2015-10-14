/*
    ICE Telescope ROS Package: Meade LX200GPS Telescope Server
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
#include <string>
using namespace std;

extern "C"
{
  #include "ice_telescope/lx200gps.h"
}

void meade_output(ice_telescope::meade::Response &res, string out_str, bool error)
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

bool meade_action(ice_telescope::meade::Request  &req,
                      ice_telescope::meade::Response &res)
{
  int portFD = -1;

  if(lx200_connect(&portFD))
  {
    if(req.meade_action == "gps")
    {
      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

      gpsRestart(portFD);
      updateGPS_System(portFD);

      meade_output(res, "Updating gps", false);
    }
    else if(req.meade_action == "getobjradec")
    {
      double obj_ra, obj_dec;

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

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
    else if(req.meade_action == "gettelradec")
    {
      double lx_ra, lx_dec;

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

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
    else if(req.meade_action == "getdatetime")
    {
      char ltime[64];
      char date[64];

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

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
    else if(req.meade_action == "setdatetime")
    {
      struct tm* pTm;
      struct timeval tv;
      struct timezone tz;

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

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
    else if(req.meade_action == "getlatlon")
    {
      int dd, mm;
      int ddd, mmm;

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

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
    else if(req.meade_action == "setlatlon")
    {
      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

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
    else if(req.meade_action == "focus")
    {
      int motion_type;

      ROS_INFO("Telescope request: %s %s", req.meade_action.c_str(), req.focus_motion.c_str());

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
        return true;
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
    else if(req.meade_action == "goto")
    {
      char* return_msg;

      ROS_INFO("Telescope request: %s RA %f - DEC %f", req.meade_action.c_str(), req.ra, req.dec);
      
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
    else if(req.meade_action == "messier" || req.meade_action == "star" || req.meade_action == "deepsky")
    {
      int catalog = 0;
      int err = 0;

      ROS_INFO("Telescope request: %s %d", req.meade_action.c_str(), req.object_num);

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "meade_server");
  ros::NodeHandle n;

  // Telescope service
  ros::ServiceServer meadeService = n.advertiseService("meade_action", meade_action);
  ROS_INFO("Ready to control telescope");

  ros::spin();

  return 0;
}
