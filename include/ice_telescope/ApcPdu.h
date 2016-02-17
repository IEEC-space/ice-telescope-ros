/*
    ICE Telescope ROS Package: APC Switched PDU
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

#include "ice_telescope/apc.h"
#include <string>
using namespace std;

class ApcPdu
{

public:
  ApcPdu();
  ~ApcPdu();

  bool apc_action(ice_telescope::apc::Request &req, ice_telescope::apc::Response &res);

protected:
  bool apc_reconnect(ice_telescope::apc::Response &res);
  void apc_input(ice_telescope::apc::Request &req);
  void apc_output(ice_telescope::apc::Response &res, string out_str, bool error);
  void apc_action_power(ice_telescope::apc::Request &req, ice_telescope::apc::Response &res);
  void apc_action_status(ice_telescope::apc::Request &req, ice_telescope::apc::Response &res);

};
