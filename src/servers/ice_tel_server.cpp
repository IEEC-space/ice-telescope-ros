/*
    ICE Telescope ROS Package: Full Server (dome, telescope, ccd)
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
#include "ice_telescope/ApcPdu.h"
#include "ice_telescope/BaaderDome.h"
#include "ice_telescope/MeadeTelescope.h"
#include "ice_telescope/SbigCcd.h"
#include "ice_telescope/VaisalaWS.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_tel_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(12); // Use 8 threads. 0 -> use the number of cores.

  // Power distribution service
  ApcPdu apcPdu;
  ros::ServiceServer apcService = n.advertiseService("apc_action", &ApcPdu::apc_action, &apcPdu);

  // Dome service
  BaaderDome baaderDome;
  ros::ServiceServer baaderService = n.advertiseService("baader_action", &BaaderDome::baader_action, &baaderDome);

  // Telescope service
  MeadeTelescope meadeTelescope;
  ros::ServiceServer meadeService = n.advertiseService("meade_action", &MeadeTelescope::meade_action, &meadeTelescope);

  // CCD service
  SbigCcd sbigCcd;
  ros::ServiceServer sbigService = n.advertiseService("sbig_action", &SbigCcd::sbig_action, &sbigCcd);

  // Weather station service
  VaisalaWS vaisalaWS;
  ros::ServiceServer vaisalaService = n.advertiseService("vaisala_action", &VaisalaWS::vaisala_action, &vaisalaWS);

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
