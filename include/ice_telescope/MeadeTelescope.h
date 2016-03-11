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

#include "ice_telescope/meade.h"
#include <string>
using namespace std;

class MeadeTelescope
{

public:
  MeadeTelescope();
  ~MeadeTelescope();

  bool meade_action(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res);

protected:
  int portFD;

  bool meade_reconnect(ice_telescope::meade::Response &res);
  void meade_input(ice_telescope::meade::Request &req, string in_str);
  void meade_output(ice_telescope::meade::Response &res, string out_str, bool error);
  void meade_action_gps(ice_telescope::meade::Response &res);
  void meade_action_getobjradec(ice_telescope::meade::Response &res);
  void meade_action_gettelradec(ice_telescope::meade::Response &res);
  void meade_action_getdatetime(ice_telescope::meade::Response &res);
  void meade_action_setdatetime(ice_telescope::meade::Response &res);
  void meade_action_getlatlon(ice_telescope::meade::Response &res);
  void meade_action_setlatlon(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res);
  void meade_action_focus(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res);
  void meade_action_goto(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res);
  void meade_action_catalog(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res);
  void meade_action_park(ice_telescope::meade::Response &res);
  void meade_action_status(ice_telescope::meade::Response &res);
  void meade_action_init(ice_telescope::meade::Response &res);
  void meade_action_move(ice_telescope::meade::Request &req, ice_telescope::meade::Response &res);

};