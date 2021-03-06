/*
    ICE Telescope ROS Package: SBIG CCD Server
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
#include "ice_telescope/SbigCcd.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbig_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(4);

  // CCD service
  SbigCcd sbigCcd;
  ros::ServiceServer sbigService = n.advertiseService("sbig_action", &SbigCcd::sbig_action, &sbigCcd);

  spinner.start();
  ros::waitForShutdown();

  return 0;
}