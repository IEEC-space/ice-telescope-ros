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
#include "ice_telescope/BaaderDome.h"
#include "ice_telescope/MeadeTelescope.h"
#include "ice_telescope/SbigCcd.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_tel_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(4); // Use 4 threads. 0 -> use the number of cores.

  // Dome service
  BaaderDome baaderDome;
  ros::ServiceServer baaderService = n.advertiseService("baader_action", &BaaderDome::baader_action, &baaderDome);
  ROS_INFO("Ready to control dome");

  // Telescope service
  MeadeTelescope meadeTelescope;
  ros::ServiceServer meadeService = n.advertiseService("meade_action", &MeadeTelescope::meade_action, &meadeTelescope);
  ROS_INFO("Ready to control telescope");

  // CCD service
  SbigCcd sbigCcd;
  ros::ServiceServer sbigService = n.advertiseService("sbig_action", &SbigCcd::sbig_action, &sbigCcd);
  ROS_INFO("Ready to control ccd");

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
