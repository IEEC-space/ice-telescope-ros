/*
    ICE Telescope ROS Package: Vaisala Weather Station
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

#include "ice_telescope/vaisala.h"
#include <string>
using namespace std;

class VaisalaWS
{

public:
  VaisalaWS();
  ~VaisalaWS();

  bool vaisala_action(ice_telescope::vaisala::Request &req, ice_telescope::vaisala::Response &res);

protected:
  int portFD;

  bool vaisala_reconnect(ice_telescope::vaisala::Response &res);
  void vaisala_input(ice_telescope::vaisala::Request &req);
  void vaisala_output(ice_telescope::vaisala::Response &res, string out_str, bool error);
  void vaisala_action_getinfo(ice_telescope::vaisala::Response &res);
  void vaisala_action_reset(ice_telescope::vaisala::Response &res);
  
};