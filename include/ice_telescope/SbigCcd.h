/*
    ICE Telescope ROS Package: SBIG CCD
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

#include "ice_telescope/sbig.h"
#include "ice_telescope/lpardrv.h"
#include "ice_telescope/csbigcam.h"
#include "ice_telescope/csbigimg.h"
#include <string>
using namespace std;

class SbigCcd
{

public:
  SbigCcd();
  ~SbigCcd();

  bool sbig_action(ice_telescope::sbig::Request &req, ice_telescope::sbig::Response &res);

protected:
  bool sbig_connect();
  bool sbig_disconnect();
  bool sbig_reconnect(ice_telescope::sbig::Response &res);
  void sbig_input(ice_telescope::sbig::Request &req);
  void sbig_output(ice_telescope::sbig::Response &res, CSBIGCam* pCam, string out_str, bool error, PAR_ERROR err);
  void sbig_action_gettemp(ice_telescope::sbig::Response &res, CSBIGCam* pCam, PAR_ERROR err);
  void sbig_action_getcapstatus(ice_telescope::sbig::Response &res, CSBIGCam* pCam);
  void sbig_action_settemp(ice_telescope::sbig::Request &req, ice_telescope::sbig::Response &res, CSBIGCam* pCam, PAR_ERROR err);
  void sbig_action_capture(ice_telescope::sbig::Request &req, ice_telescope::sbig::Response &res, CSBIGCam* pCam, CSBIGImg* pImg, PAR_ERROR err);

private:
  CSBIGImg* pImg;
  CSBIGCam* pCam;
  PAR_ERROR err;

};
