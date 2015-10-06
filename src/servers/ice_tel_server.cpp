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

#include <libusb.h>

#include "ros/ros.h"
#include "ice_telescope/baader.h"
#include "ice_telescope/sbig.h"
#include "ice_telescope/meade.h"

#include "ice_telescope/lpardrv.h"
#include "ice_telescope/csbigcam.h"
#include "ice_telescope/csbigimg.h"

extern "C"
{
  #include "ice_telescope/baader_dome.h"
  #include "ice_telescope/lx200gps.h"
}


void baader_output(ice_telescope::baader::Response &res, string out_str, bool error)
{
  res.baader_response = out_str;
  res.baader_error = error;
  if(error)
  {
    ROS_ERROR(res.baader_response.c_str());
  }
  else
  {
    ROS_INFO(res.baader_response.c_str());
  }
}

bool baader_action(ice_telescope::baader::Request  &req,
                  ice_telescope::baader::Response &res)
{
  if(dome_connect())
  {
    //ROS_INFO("Dome connected");
    ROS_INFO("Dome request: %s", req.baader_action.c_str());

    if(req.baader_action == "open")
    {
      if(dome_control_shutter(SHUTTER_OPEN)) // Open shutter
      {
        baader_output(res, "Opening shutter... This process can take up to 60 seconds", false);
      }
      else
      {
        baader_output(res, "Error opening shutter", true);
      }
    }
    else if(req.baader_action == "close")
    {
      if(dome_control_shutter(SHUTTER_CLOSE)) // Close shutter
      {
        baader_output(res, "Closing shutter... This process can take up to 60 seconds", false);
      }
      else
      {
        baader_output(res, "Error closing shutter", true);
      }
    }
    else if(req.baader_action == "status")
    {
      if(dome_shutter_status()) // Shutter status
      {
        baader_output(res, dome_get_shutter_status_string(shutterStatus), false);
      }
      else
      {
        baader_output(res, "Error getting shutter status", true);
      }
    }
    else
    {
      baader_output(res, "Invalid dome action", true);
    }

    dome_disconnect();
    //ROS_INFO("Dome disconnected");
  }
  else
  {
    baader_output(res, "Dome connection failed", true);
  }

  return true;
}

void sbig_output_error(CSBIGCam* pCam, ice_telescope::sbig::Response &res, PAR_ERROR err, string err_str)
{
  if(err != CE_NO_ERROR)
  {
    res.sbig_response = err_str + pCam->GetErrorString(err);
  }
  else
  {
    res.sbig_response = err_str;
  }
  res.sbig_error = true;
  ROS_ERROR(res.sbig_response.c_str());
}

bool sbig_action(ice_telescope::sbig::Request  &req,
                  ice_telescope::sbig::Response &res)
{
  PAR_ERROR err = CE_NO_ERROR;

  // Create SBIG Img object
  CSBIGImg* pImg = new CSBIGImg;

  // Create SBIG camera object
  CSBIGCam* pCam = new CSBIGCam(DEV_USB1);

  if((err = pCam->GetError()) != CE_NO_ERROR)
  {
    sbig_output_error(pCam, res, err, "CSBIGCam error: ");
    return true;
  }

  // Establish link to the camera
  if((err = pCam->EstablishLink()) != CE_NO_ERROR)
  {
    sbig_output_error(pCam, res, err, "Establish link error: ");
    return true;
  }

  //ROS_INFO("Link established to: %s", pCam->GetCameraTypeString().c_str());

  res.sbig_error = false;

  if(req.sbig_action == "gettemp")
  {
    double sbigTemp, setpointTemp, percentTE;
    MY_LOGICAL enabled;

    ROS_INFO("Getting CCD temperature");

    if((err = pCam->QueryTemperatureStatus(enabled, sbigTemp, setpointTemp, percentTE)) != CE_NO_ERROR)
    {
      sbig_output_error(pCam, res, err, "Get temperature error: ");
      return true;
    }

    std::stringstream s;
    s << "CCD temperature is " << sbigTemp << " degrees C. Power to cooler: " << percentTE << ". Enabled: " << enabled;
    res.sbig_response = s.str();
    ROS_INFO(res.sbig_response.c_str());
  }
  else if(req.sbig_action == "settemp")
  {
    ROS_INFO("Setting CCD temperature");

    if((err = pCam->SetTemperatureRegulation(req.temp_enable, req.temperature)) != CE_NO_ERROR)
    {
      sbig_output_error(pCam, res, err, "Set temperature error: ");
      return true;
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
    res.sbig_response = s.str();
    ROS_INFO(res.sbig_response.c_str());
  }
  else if(req.sbig_action == "capture")
  {
    SBIG_FILE_ERROR ferr;
    int fullWidth, fullHeight;
    string filePathName;

    ROS_INFO("CCD exposure request");

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
          sbig_output_error(pCam, res, err, "CSBIGCam error: ");
          break;
        }
      }
      else
      {
        ROS_INFO("Taking dark frame no.: %d", i);
        if((err = pCam->GrabImage(pImg, SBDF_DARK_ONLY)) != CE_NO_ERROR)
        {
          sbig_output_error(pCam, res, err, "CSBIGCam error: ");
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
          sbig_output_error(pCam, res, CE_NO_ERROR, "SBIF_FITS format save error: " + ferr);
          break;
        }
      }
      else
      {
        filePathName += ".sbig";
        if((ferr = pImg->SaveImage(filePathName.c_str(), SBIF_COMPRESSED)) != SBFE_NO_ERROR)
        {
          sbig_output_error(pCam, res, CE_NO_ERROR, "SBIF_COMPRESSED format save error: " + ferr);
          break;
        }
      }

      ROS_INFO("File saved as: %s", filePathName.c_str());
    }

    res.sbig_response = "File(s) saved to disk";
  }
  else
  {
    sbig_output_error(NULL, res, CE_NO_ERROR, "Invalid CCD action");
  }

  // Close sbig device
  if((err = pCam->CloseDevice()) != CE_NO_ERROR)
  {
    sbig_output_error(pCam, res, err, "CSBIGCam error: ");
    return true;
  }

  // Close sbig driver
  if((err = pCam->CloseDriver()) != CE_NO_ERROR)
  {
    sbig_output_error(pCam, res, err, "CSBIGCam error: ");
    return true;
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

void meade_output(ice_telescope::meade::Response &res, string out_str, bool error)
{
  res.meade_response = out_str;
  res.meade_error = error;
  if(error)
  {
    ROS_ERROR(res.meade_response.c_str());
  }
  else
  {
    ROS_INFO(res.meade_response.c_str());
  }
}

bool meade_action(ice_telescope::meade::Request  &req,
                      ice_telescope::meade::Response &res)
{
  int portFD = -1;

  if(lx200_connect(&portFD))
  {
    if(req.meade_action == "gps")
    {
      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

      gpsRestart(portFD);
      updateGPS_System(portFD);

      meade_output(res, "Updating gps", false);
    }
    else if(req.meade_action == "getobjradec")
    {
      double obj_ra, obj_dec;

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

      if(getObjectRA(portFD, &obj_ra) == 0 && getObjectDEC(portFD, &obj_dec) == 0)
      {
        std::stringstream s;
        s << "Target object is set to RA " << obj_ra << " - DEC " << obj_dec;
        meade_output(res, s.str(), false);
      }
      else
      {
        meade_output(res, "Error getting object's RA/DEC", true);
      }
    }
    else if(req.meade_action == "gettelradec")
    {
      double lx_ra, lx_dec;

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

      if(getLX200RA(portFD, &lx_ra) == 0 && getLX200DEC(portFD, &lx_dec) == 0)
      {
        std::stringstream s;
        s << "Telescope is at RA " << lx_ra << " - DEC " << lx_dec;
        meade_output(res, s.str(), false);
      }
      else
      {
        meade_output(res, "Error getting telescope's RA/DEC", true);
      }
    }
    else if(req.meade_action == "getdatetime")
    {
      char ltime[64];
      char date[64];

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

      if(getCalenderDate(portFD, date) == 0 && getLocTime(portFD, ltime) == 0)
      {
        std::stringstream s;
        s << "Telescope date and time: " << date << " " << ltime;
        meade_output(res, s.str(), false);
      }
      else
      {
        meade_output(res, "Error getting telescope's date and time", true);
      }
    }
    else if(req.meade_action == "setdatetime")
    {
      struct tm* pTm;
      struct timeval tv;
      struct timezone tz;

      ROS_INFO("Telescope request: %s", req.meade_action.c_str());

      gettimeofday(&tv, &tz);
      pTm = localtime(&(tv.tv_sec));

      if(setCalenderDate(portFD, pTm->tm_mday, pTm->tm_mon + 1, pTm->tm_year + 1900) == 0 && setLocalTime(portFD, pTm->tm_hour, pTm->tm_min, pTm->tm_sec) == 0)
      {
        meade_output(res, "Telescope's date and time set to now", false);
      }
      else
      {
        meade_output(res, "Error setting telescope's date and time", true);
      }
    }
    else if(req.meade_action == "focus")
    {
      int motion_type;

      ROS_INFO("Telescope request: %s %s", req.meade_action.c_str(), req.focus_motion.c_str());

      // if(req.focus_motion == "in")
      // {
      //   motion_type = LX200_FOCUSIN;
      // }
      // else if(req.focus_motion == "out")
      // {
      //   motion_type = LX200_FOCUSOUT;
      // }
      // else
      // {
        meade_output(res, "Invalid focus motion. Options are in/out", true);
        return true;
      // }

      // if(setFocuserMotion(portFD, motion_type) == 0)
      // {
      //   std::stringstream s;
      //   s << "Telescope focusing " << req.focus_motion;
      //   meade_output(res, s.str(), false);
      // }
      // else
      // {
      //   meade_output(res, "Error focusing telescope", true);
      // }
    }
    else if(req.meade_action == "goto")
    {
      char* return_msg;

      ROS_INFO("Telescope request: %s RA %f - DEC %f", req.meade_action.c_str(), req.ra, req.dec);
      
      return_msg = (char*)malloc(64*sizeof(char));
      if(GoTo(portFD, req.ra, req.dec, return_msg))
      {
        meade_output(res, "Telescope slewing to target", false);
      }
      else
      {
        meade_output(res, string(return_msg), true);
      }
    }
    else if(req.meade_action == "messier" || req.meade_action == "star" || req.meade_action == "deepsky")
    {
      int catalog = 0;
      int err = 0;

      ROS_INFO("Telescope request: %s %d", req.meade_action.c_str(), req.object_num);

      if(req.meade_action == "messier")
        catalog = LX200_MESSIER_C;
      else if(req.meade_action == "star")
        catalog = LX200_STAR_C;
      else
        catalog = LX200_DEEPSKY_C;

      if(selectCatalogObject(portFD, catalog, req.object_num) == 0)
      {
        if((err = Slew(portFD)) == 0)
        {
          meade_output(res, "Telescope slewing to target", false);
        }
        else
        {
          meade_output(res, string(slewErrorString(err)), true);
        }
      }
      else
      {
        meade_output(res, "Target selection failed", true);
      }
    }
    else
    {
      meade_output(res, "Invalid telescope action", true);
    }

    lx200_disconnect(portFD);
  }
  else
  {
    meade_output(res, "Telescope connection failed", true);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_tel_server");
  ros::NodeHandle n;

  // Dome service
  ros::ServiceServer baaderService = n.advertiseService("baader_action", baader_action);
  ROS_INFO("Ready to control dome");

  // CCD service
  ros::ServiceServer sbigService = n.advertiseService("sbig_action", sbig_action);
  ROS_INFO("Ready to control ccd");

  // Telescope service
  ros::ServiceServer meadeService = n.advertiseService("meade_action", meade_action);
  ROS_INFO("Ready to control telescope");

  ros::spin();

  return 0;
}
