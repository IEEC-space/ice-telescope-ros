/*
    ICE Telescope ROS Package: Vaisala Weather Station driver
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

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>

#ifndef _WIN32
#include <termios.h>
#endif

#include "ice_telescope/tty_com.h"
#include "ice_telescope/vaisala_ws.h"

#define DEVICE_NAME "/dev/ttyUSB3"
#define WS_BUF 32
#define WS_TIMEOUT 3

bool ws_connect(int *in_fd)
{
  int portFD = -1;
  int connectrc = 0;
  char errorMsg[MAXRBUF];

  if((connectrc = tty_connect(DEVICE_NAME, 9600, 8, 0, 1, &portFD)) != TTY_OK)
  {
    tty_error_msg(connectrc, errorMsg, MAXRBUF);
    if(tty_debug) fprintf(stderr, "Failed to connect to port %s. %s\n", DEVICE_NAME, errorMsg);
    return false;
  }

  if(ws_ack(portFD))
  {
    if(tty_debug) fprintf(stderr, "Weather station is online\n");
    *in_fd = portFD;
    return true;
  }

  if(tty_debug) fprintf(stderr, "Error retreiving data from weather station, please ensure dome controller is powered and the port is correct.\n");
  return false;
}

bool ws_disconnect(int portFD)
{
  tty_disconnect(portFD);
  if(tty_debug) fprintf(stderr, "Weather station is offline\n");
  return true;
}

bool ws_ack(int portFD)
{
  int nbytes_written=0, nbytes_read=0, rc=-1;
  char errstr[MAXRBUF];
  char resp[WS_BUF];
  char status[WS_BUF];

  tcflush(portFD, TCIOFLUSH);

  if((rc = tty_write(portFD, "vers\r", 5, &nbytes_written)) != TTY_OK)
  {
    tty_error_msg(rc, errstr, MAXRBUF);
    if(tty_debug) fprintf(stderr, "vers Ack error: %s\n", errstr);
    return false;
  }

  if(tty_debug) fprintf(stderr, "CMD (vers)\n");

  usleep(100000);

  if((rc = tty_read(portFD, resp, 16, WS_TIMEOUT, &nbytes_read)) != TTY_OK)
  {
    tty_error_msg(rc, errstr, MAXRBUF);
    if(tty_debug) fprintf(stderr, "Ack error: %s\n", errstr);
    return false;
  }
  tcflush(portFD, TCIFLUSH);

  if(rc > 0)
    return true;

  resp[nbytes_read] = '\0';

  if(tty_debug) fprintf(stderr, "RES (%s)\n", resp);

  rc = sscanf(resp, "%s", status);
  if(rc > 0)
    return true;
  else
    return false;
}

bool ws_getinfo(int portFD, char *info)
{
  int error_type, i;
  int nbytes_write=0, nbytes_read=0;
  char resp[64];
  float hpa, temp, rh;

  for(i=0; i<64; i++) resp[i] = '\0';
  tcflush(portFD, TCIOFLUSH);

  if(tty_debug)
    fprintf(stderr, "%s Command [send]\n", __FUNCTION__);

  if((error_type = tty_write(portFD, "send\r", 5, &nbytes_write)) != TTY_OK)
    return false;

  usleep(100000);

  error_type = tty_read(portFD, resp, 31, WS_TIMEOUT, &nbytes_read);

  tcflush(portFD, TCIFLUSH);

  if (error_type != TTY_OK)
    return false;

  if(sscanf(resp, "%f %*s %f %*s %f %*s", &hpa, &temp, &rh) < 3)
    return false;
  else
    snprintf(info, sizeof(resp), "%.1f hPa %.2f 'C %.2f %%RH", hpa, temp, rh);

  if(tty_debug)
    fprintf(stderr, "%s Response <%s>\n", __FUNCTION__, info);
  
  return true;
}

bool ws_reset(int portFD)
{
  int nbytes_written=0, nbytes_read=0, rc=-1;
  char errstr[MAXRBUF];
  char resp[WS_BUF];
  char status[WS_BUF];

  tcflush(portFD, TCIOFLUSH);

  if((rc = tty_write(portFD, "reset\r", 6, &nbytes_written)) != TTY_OK)
  {
    tty_error_msg(rc, errstr, MAXRBUF);
    if(tty_debug) fprintf(stderr, "reset Ack error: %s\n", errstr);
    return false;
  }

  if(tty_debug) fprintf(stderr, "CMD (reset)\n");

  usleep(100000);

  if((rc = tty_read(portFD, resp, 15, WS_TIMEOUT, &nbytes_read)) != TTY_OK)
  {
    tty_error_msg(rc, errstr, MAXRBUF);
    if(tty_debug) fprintf(stderr, "Ack error: %s\n", errstr);
    return false;
  }

  resp[nbytes_read] = '\0';

  if(tty_debug) fprintf(stderr, "RES (%s)\n", resp);

  rc = sscanf(resp, "%s", status);
  if(rc > 0)
    return true;
  else
    return false;
}

