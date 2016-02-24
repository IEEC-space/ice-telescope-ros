/*
    ICE Telescope ROS Package: Vaisala Weather Station Server
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vaisala_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(4);

  // Weather station service
  VaisalaWS vaisalaWS;
  ros::ServiceServer vaisalaService = n.advertiseService("vaisala_action", &VaisalaWS::vaisala_action, &vaisalaWS);

  spinner.start();
  ros::waitForShutdown();

  return 0;
}