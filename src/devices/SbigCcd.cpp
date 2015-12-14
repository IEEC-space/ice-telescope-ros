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

#include "ros/ros.h"
#include "ice_telescope/SbigCcd.h"
    

SbigCcd::SbigCcd()
:err(CE_NO_ERROR)
{
  if(sbig_connect())
  {
    ROS_INFO("Ready to control CCD");
  }
  else
  {
    ROS_ERROR("CCD connection failed. Check power and retry");
  }
}

SbigCcd::~SbigCcd()
{
  sbig_disconnect();
}

bool SbigCcd::sbig_connect()
{
  // Create SBIG Img object
  pImg = new CSBIGImg;

  // Create SBIG camera object
  pCam = new CSBIGCam(DEV_USB1);

  ROS_INFO("Connecting to CCD");

  if((err = pCam->GetError()) != CE_NO_ERROR)
  {
    ROS_ERROR("CSBIGCam error: %s", pCam->GetErrorString(err).c_str());
    return false;
  }

  // Establish link to the camera
  if((err = pCam->EstablishLink()) != CE_NO_ERROR)
  {
    ROS_ERROR("Establish link error: %s", pCam->GetErrorString(err).c_str());
    return false;
  }

  return true;
}

bool SbigCcd::sbig_disconnect()
{
  // Close sbig device
  if(pCam && (err = pCam->CloseDevice()) != CE_NO_ERROR)
  {
    ROS_ERROR("CSBIGCam error: %s", pCam->GetErrorString(err).c_str());
  }

  // Close sbig driver
  if(pCam && (err = pCam->CloseDriver()) != CE_NO_ERROR)
  {
    ROS_ERROR("CSBIGCam error: %s", pCam->GetErrorString(err).c_str());
  }

  // Delete objects
  if(pImg)
  {
    delete pImg;
  }

  if(pCam)
  {
    delete pCam;
  }

  return true;
}

bool SbigCcd::sbig_reconnect(ice_telescope::sbig::Response &res)
{
  err = CE_NO_ERROR;

  sbig_disconnect();
  if(sbig_connect())
  {
    sbig_output(res, NULL, "Ready to control CCD", false, CE_NO_ERROR);
    return true;
  }

  sbig_output(res, NULL, "CCD connection failed. Check power and retry", true, CE_NO_ERROR);
  return false;
}

bool SbigCcd::sbig_action(ice_telescope::sbig::Request &req, ice_telescope::sbig::Response &res)
{
  sbig_input(req);

  if(pCam->CheckLink() == FALSE)
  {
    ROS_ERROR("CCD disconnected");
    if(!sbig_reconnect(res))
    {
      return true;
    }
  }

  if(req.sbig_action == "gettemp")
  {
    sbig_action_gettemp(res, pCam, err);
  }
  else if(req.sbig_action == "getcapstatus")
  {
    sbig_action_getcapstatus(res, pCam);
  }
  else if(req.sbig_action == "settemp")
  {
    sbig_action_settemp(req, res, pCam, err);
  }
  else if(req.sbig_action == "capture")
  {
    sbig_action_capture(req, res, pCam, pImg, err);
  }
  else if(req.sbig_action == "reconnect")
  {
    sbig_reconnect(res);
  }
  else
  {
    sbig_output(res, NULL, "Invalid CCD action", true, CE_NO_ERROR);
  }

  return true;
}

void SbigCcd::sbig_input(ice_telescope::sbig::Request &req)
{
  ROS_INFO("CCD request: %s", req.sbig_action.c_str());
}

void SbigCcd::sbig_output(ice_telescope::sbig::Response &res, CSBIGCam* pCam, string out_str, bool error, PAR_ERROR err)
{
  res.sbig_error = error;
  if(err != CE_NO_ERROR)
  {
    res.sbig_response = out_str + pCam->GetErrorString(err);
  }
  else
  {
    res.sbig_response = out_str;
  }

  if(error)
  {
    ROS_ERROR(res.sbig_response.c_str());
  }
  else
  {
    ROS_INFO(res.sbig_response.c_str());
  }
}

void SbigCcd::sbig_action_gettemp(ice_telescope::sbig::Response &res, CSBIGCam* pCam, PAR_ERROR err)
{
  double sbigTemp, setpointTemp, percentTE;
  MY_LOGICAL enabled;

  if((err = pCam->QueryTemperatureStatus(enabled, sbigTemp, setpointTemp, percentTE)) != CE_NO_ERROR)
  {
    sbig_output(res, pCam, "Get temperature error: ", true, err);
    return;
  }

  std::stringstream s;
  s << "CCD temperature is " << sbigTemp << " degrees C. Power to cooler: " << percentTE << ". Enabled: " << enabled;
  sbig_output(res, NULL, s.str(), false, CE_NO_ERROR);
}

void SbigCcd::sbig_action_getcapstatus(ice_telescope::sbig::Response &res, CSBIGCam* pCam)
{
  GRAB_STATE grabState;
  double percentComplete;

  pCam->GetGrabState(grabState, percentComplete);

  std::stringstream s;
  if(grabState != GS_IDLE)
  {
    s << "CCD is busy. Progress: " << (percentComplete * 100) << "%%";
  }
  else
  {
    s << "CCD is IDLE"; 
  }
  sbig_output(res, NULL, s.str(), false, CE_NO_ERROR);
}

void SbigCcd::sbig_action_settemp(ice_telescope::sbig::Request &req, ice_telescope::sbig::Response &res, CSBIGCam* pCam, PAR_ERROR err)
{
  if((err = pCam->SetTemperatureRegulation(req.temp_enable, req.temperature)) != CE_NO_ERROR)
  {
    sbig_output(res, pCam, "Set temperature error: ", true, err);
    return;
  }

  std::stringstream s;
  if(req.temp_enable)
  {
    s << "CCD temperature set to " << req.temperature << " degrees C";
  }
  else
  {
    s << "CCD temperature regulation disabled";
  }
  sbig_output(res, NULL, s.str(), false, CE_NO_ERROR);
}

void SbigCcd::sbig_action_capture(ice_telescope::sbig::Request &req, ice_telescope::sbig::Response &res, CSBIGCam* pCam, CSBIGImg* pImg, PAR_ERROR err)
{
  SBIG_FILE_ERROR ferr;
  int fullWidth, fullHeight;
  string filePathName;

  if(req.lf_img)
  {
    req.file_path += "LF_";
  }
  else
  {
    req.file_path += "DF_";
  }

  // Set camera params
  pCam->SetActiveCCD(CCD_IMAGING);
  pCam->SetExposureTime(req.exp_time);
  pCam->SetReadoutMode(req.readout_mode);
  pCam->SetABGState(ABG_LOW7);
  pCam->SetFastReadout(req.fast_readout);
  pCam->SetDualChannelMode(req.dual_readout_channel);

  // Update width and height of the image
  pCam->GetFullFrame(fullWidth, fullHeight);

  if(req.width == 0)
  {
    req.width = fullWidth;
  }

  if(req.height == 0)
  {
    req.height = fullHeight;
  }

  pCam->SetSubFrame(req.left, req.top, req.width, req.height);
  pImg->AllocateImageBuffer(req.height, req.width);

  // Take series of images
  for(int i = 1; i <= req.img_count; ++i)
  {
    if(req.lf_img)
    {
      ROS_INFO("Taking light frame no.: %d", i);
      if((err = pCam->GrabImage(pImg, SBDF_LIGHT_ONLY)) != CE_NO_ERROR)
      {
        sbig_output(res, pCam, "CSBIGCam error: ", true, err);
        break;
      }
    }
    else
    {
      ROS_INFO("Taking dark frame no.: %d", i);
      if((err = pCam->GrabImage(pImg, SBDF_DARK_ONLY)) != CE_NO_ERROR)
      {
        sbig_output(res, pCam, "CSBIGCam error: ", true, err);
        break;
      }
    }

    char timeBuf[128];
    struct tm* pTm;

    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    pTm = localtime(&(tv.tv_sec));
    sprintf(timeBuf, "%04d-%02d-%02dT%02d:%02d:%02d.%03ld",
        pTm->tm_year + 1900, pTm->tm_mon + 1, pTm->tm_mday, pTm->tm_hour,
        pTm->tm_min, pTm->tm_sec, (tv.tv_usec / 1000));

    filePathName = req.file_path + timeBuf;

    if(req.fits_file)
    {
      filePathName += ".fits";
      if((ferr = pImg->SaveImage(filePathName.c_str(), SBIF_FITS)) != SBFE_NO_ERROR)
      {
        sbig_output(res, pCam, "SBIF_FITS format save error: " + ferr, true, CE_NO_ERROR);
        break;
      }
    }
    else
    {
      filePathName += ".sbig";
      if((ferr = pImg->SaveImage(filePathName.c_str(), SBIF_COMPRESSED)) != SBFE_NO_ERROR)
      {
        sbig_output(res, pCam, "SBIF_COMPRESSED format save error: " + ferr, true, CE_NO_ERROR);
        break;
      }
    }

    ROS_INFO("File saved as: %s", filePathName.c_str());
  }

  sbig_output(res, NULL, "File(s) saved to disk", false, CE_NO_ERROR);
}
