/*
    ICE Telescope ROS Package: Baader Planetarium Dome
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

#include "ice_telescope/baader.h"
#include <string>
using namespace std;

class BaaderDome
{

public:
  BaaderDome();
  ~BaaderDome();

  bool baader_action(ice_telescope::baader::Request &req, ice_telescope::baader::Response &res);

protected:
  void baader_input(ice_telescope::baader::Request &req);
  void baader_output(ice_telescope::baader::Response &res, string out_str, bool error);
  void baader_action_open(ice_telescope::baader::Response &res);
  void baader_action_close(ice_telescope::baader::Response &res);
  void baader_action_status(ice_telescope::baader::Response &res);
    
};