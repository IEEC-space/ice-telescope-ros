/*
    ICE Telescope ROS Package: APC Switched PDU
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
#include "ice_telescope/ApcPdu.h"

extern "C"
{
  #include "ice_telescope/snmp_com.h"
}

const char* PDU_HOST = "10.50.1.213";
const char* TELESCOPE_OID = ".1.3.6.1.4.1.318.1.1.12.3.3.1.1.4.2";
const char* CCD_OID = ".1.3.6.1.4.1.318.1.1.12.3.3.1.1.4.3";
const char* WS_OID = ".1.3.6.1.4.1.318.1.1.12.3.3.1.1.4.4";
const char* LIGHT_OID = ".1.3.6.1.4.1.318.1.1.12.3.3.1.1.4.5";


ApcPdu::ApcPdu()
{
  ROS_INFO("Connecting to PDU");
  if(snmpc_open(PDU_HOST))
  {
    ROS_INFO("Ready to control PDU");
  }
  else
  {
    ROS_ERROR("PDU connection failed. Check power and retry");
  }
}

ApcPdu::~ApcPdu()
{
  snmpc_close();
}

bool ApcPdu::apc_reconnect(ice_telescope::apc::Response &res)
{
  snmpc_close();
  ROS_INFO("Connecting to PDU");
  if(snmpc_open(PDU_HOST))
  {
    apc_output(res, "Ready to control PDU", false);
    return true;
  }
  
  apc_output(res, "PDU connection failed. Check power and retry", true);
  return false;
}

bool ApcPdu::apc_action(ice_telescope::apc::Request &req, ice_telescope::apc::Response &res)
{
  apc_input(req);

  if(!snmpc_check_session())
  {
    ROS_ERROR("PDU disconnected");
    if(!apc_reconnect(res))
    {
      return true;
    }
  }

  if(req.apc_action == "power_on" || req.apc_action == "power_off")
  {
    apc_action_power(req, res);
  }
  else if(req.apc_action == "power_status")
  {
    apc_action_status(req, res);
  }
  else
  {
    apc_output(res, "Invalid PDU action", true);
  }

  return true;
}

void ApcPdu::apc_input(ice_telescope::apc::Request &req)
{
  ROS_INFO("PDU request: %s %s", req.apc_action.c_str(), req.apc_device.c_str());
}
  
void ApcPdu::apc_output(ice_telescope::apc::Response &res, string out_str, bool error)
{
  res.apc_response = out_str;
  res.apc_error = error;
  if(error)
  {
    ROS_ERROR(res.apc_response.c_str());
  }
  else
  {
    ROS_INFO(res.apc_response.c_str());
  }
}

void ApcPdu::apc_action_power(ice_telescope::apc::Request &req, ice_telescope::apc::Response &res)
{
  char buffer[64];
  char *oid;
  std::stringstream s;

  if(req.apc_device == "telescope")
  {
    oid = (char*)TELESCOPE_OID;
  }
  else if(req.apc_device == "ccd")
  {
    oid = (char*)CCD_OID;
  }
  else if(req.apc_device == "vaisala")
  {
    oid = (char*)WS_OID;
  }
  else if(req.apc_device == "light")
  {
    oid = (char*)LIGHT_OID;
  }
  else
  {
    apc_output(res, "Invalid PDU device", true);
    return;
  }

  if(req.apc_action == "power_on")
  {
    if(snmpc_write(oid, buffer, sizeof(buffer), 'i', "1"))
    {
      s << "Power ON for the " << req.apc_device;
      apc_output(res, s.str(), false);
    }
    else
    {
      s << "Error powering on the " << req.apc_device;
      apc_output(res, s.str(), true);
    }
  }
  else if(req.apc_action == "power_off")
  {
    if(snmpc_write(oid, buffer, sizeof(buffer), 'i', "2"))
    {
      s << "Power OFF for the " << req.apc_device;
      apc_output(res, s.str(), false);
    }
    else
    {
      s << "Error powering off the " << req.apc_device;
      apc_output(res, s.str(), true);
    }
  }
}

void ApcPdu::apc_action_status(ice_telescope::apc::Request &req, ice_telescope::apc::Response &res)
{
  char buffer[64];
  char *oid;
  std::stringstream s;

  if(req.apc_device == "telescope")
  {
    oid = (char*)TELESCOPE_OID;
  }
  else if(req.apc_device == "ccd")
  {
    oid = (char*)CCD_OID;
  }
  else if(req.apc_device == "vaisala")
  {
    oid = (char*)WS_OID;
  }
  else if(req.apc_device == "light")
  {
    oid = (char*)LIGHT_OID;
  }
  else
  {
    apc_output(res, "Invalid PDU device", true);
    return;
  }

  if(snmpc_read(oid, buffer, sizeof(buffer)))
  {
    s << "Power status of " << req.apc_device << ": " << buffer;
    apc_output(res, s.str(), false);
  }
  else
  {
    s << "Error getting power status of " << req.apc_device;
    apc_output(res, s.str(), true);
  }
}

