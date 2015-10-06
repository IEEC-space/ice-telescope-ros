/*
    ICE Telescope ROS Package: SBIG CCD Server
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
#include "ice_telescope/sbig.h"
#include "ice_telescope/lpardrv.h"
#include "ice_telescope/csbigcam.h"
#include "ice_telescope/csbigimg.h"
#include <libusb.h>

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbig_server");
  ros::NodeHandle n;

  // CCD service
  ros::ServiceServer sbigService = n.advertiseService("sbig_action", sbig_action);
  ROS_INFO("Ready to control ccd");

  ros::spin();

  return 0;
}
