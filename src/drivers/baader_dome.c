/*******************************************************************************
 Baader Planetarium Dome INDI Driver

 Copyright(c) 2014 Jasem Mutlaq. All rights reserved.

 Baader Dome INDI Driver

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include <stdio.h>   /* Standard input/output definitions */
#include <stdbool.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <sys/time.h>

#ifndef _WIN32
#include <termios.h>
#endif

#include "ice_telescope/tty_com.h"
#include "ice_telescope/baader_dome.h"

#define DOME_CMD            9               /* Dome command in bytes */
#define DOME_BUF            16              /* Dome command buffer */
#define DOME_TIMEOUT        3               /* 3 seconds comm timeout */

#define DEVICE_NAME "/dev/ttyUSB0"

void dome_init_params()
{
	shutterStatus = SHUTTER_UNKNOWN;
	targetShutter = SHUTTER_CLOSE;
}

bool dome_connect(int *in_fd)
{
  int portFD = -1;
	int connectrc = 0;
	char errorMsg[MAXRBUF];

	dome_init_params();

	if((connectrc = tty_connect(DEVICE_NAME, 9600, 8, 0, 1, &portFD)) != TTY_OK)
	{
		tty_error_msg(connectrc, errorMsg, MAXRBUF);
		if(tty_debug) fprintf(stderr, "Failed to connect to port %s. %s\n", DEVICE_NAME, errorMsg);
		return false;
	}

	if(dome_ack(portFD))
	{
		if(tty_debug) fprintf(stderr, "Dome is online\n");
    *in_fd = portFD;
		return true;
	}

	if(tty_debug) fprintf(stderr, "Error retreiving data from dome, please ensure dome controller is powered and the port is correct.\n");
	return false;
}

bool dome_disconnect(int portFD)
{
	tty_disconnect(portFD);
	if(tty_debug) fprintf(stderr, "Dome is offline\n");
	return true;
}

bool dome_ack(int portFD)
{
  	int nbytes_written=0, nbytes_read=0, rc=-1;
  	char errstr[MAXRBUF];
  	char resp[DOME_BUF];
  	char status[DOME_BUF];

  	tcflush(portFD, TCIOFLUSH);

  	if((rc = tty_write(portFD, "d#getshut", DOME_CMD, &nbytes_written)) != TTY_OK)
  	{
  		tty_error_msg(rc, errstr, MAXRBUF);
  		if(tty_debug) fprintf(stderr, "d#getshut Ack error: %s\n", errstr);
  		return false;
  	}

  	if(tty_debug) fprintf(stderr, "CMD (d#getshut)\n");

    usleep(100000);

  	if((rc = tty_read(portFD, resp, DOME_CMD, DOME_TIMEOUT, &nbytes_read)) != TTY_OK)
  	{
  		tty_error_msg(rc, errstr, MAXRBUF);
        if(tty_debug) fprintf(stderr, "Ack error: %s\n", errstr);
        return false;
  	}

  	resp[nbytes_read] = '\0';

  	if(tty_debug) fprintf(stderr, "RES (%s)\n", resp);

  	rc = sscanf(resp, "d#%s", status);
  	if(rc > 0)
    	return true;
  	else
    	return false;
}

bool dome_shutter_status(int portFD)
{
  	int nbytes_written=0, nbytes_read=0, rc=-1;
  	char errstr[MAXRBUF];
  	char resp[DOME_BUF];
  	char status[DOME_BUF];

  	tcflush(portFD, TCIOFLUSH);

  	if((rc = tty_write(portFD, "d#getshut", DOME_CMD, &nbytes_written)) != TTY_OK)
  	{
  		tty_error_msg(rc, errstr, MAXRBUF);
    	if(tty_debug) fprintf(stderr, "d#getshut shutter_status error: %s\n", errstr);
    	return false;
  	}

  	if(tty_debug) fprintf(stderr, "CMD (d#getshut)\n");

    usleep(100000);

  	if((rc = tty_read(portFD, resp, DOME_CMD, DOME_TIMEOUT, &nbytes_read)) != TTY_OK)
  	{
  		tty_error_msg(rc, errstr, MAXRBUF);
    	if(tty_debug) fprintf(stderr, "shutter_status error: %s\n", errstr);
    	return false;
  	}

  	resp[nbytes_read] = '\0';

  	if(tty_debug) fprintf(stderr, "RES (%s)\n", resp);

  	rc = sscanf(resp, "d#pos%s", status);

  	if (rc > 0)
  	{
		if (!strcmp(status, "1111"))
		{
			if(shutterStatus == SHUTTER_MOVING && targetShutter == SHUTTER_OPEN)
				if(tty_debug) fprintf(stderr, "%s\n", dome_get_shutter_status_string(SHUTTER_OPENED));
			
			shutterStatus = SHUTTER_OPENED;
		}
		else if (!strcmp(status, "2222"))
		{
			if(shutterStatus == SHUTTER_MOVING && targetShutter == SHUTTER_CLOSE)
				if(tty_debug) fprintf(stderr, "%s\n", dome_get_shutter_status_string(SHUTTER_CLOSED));
			
			shutterStatus = SHUTTER_CLOSED;
		}
		else if (!strcmp(status, "1221") || !strcmp(status, "0220") || !strcmp(status, "1001"))
		{
      if(shutterStatus == SHUTTER_OPENED || shutterStatus == SHUTTER_CLOSING)
        shutterStatus = SHUTTER_CLOSING;
      else if(shutterStatus == SHUTTER_CLOSED || shutterStatus == SHUTTER_OPENING)
        shutterStatus = SHUTTER_OPENING;
      else
        shutterStatus = SHUTTER_MOVING;
		}
		else
		{
			shutterStatus = SHUTTER_MOVING;
			if(tty_debug) fprintf(stderr, "Unknown shutter status: %s\n", resp);
		}

    	return true;
  	}
  	else
    	return false;
}

bool dome_control_shutter(int portFD, ShutterOperation operation)
{
  	int nbytes_written=0, nbytes_read=0, rc=-1;
  	char errstr[MAXRBUF];
  	char cmd[DOME_BUF];
  	char resp[DOME_BUF];

  	memset(cmd, 0, sizeof(cmd));

  	if(operation == SHUTTER_OPEN)
  	{
  		targetShutter = operation;
    	strncpy(cmd, "d#opeshut", DOME_CMD);
  	}
  	else
  	{
  		targetShutter = operation;
    	strncpy(cmd, "d#closhut", DOME_CMD);
  	}

  	tcflush(portFD, TCIOFLUSH);

  	if((rc = tty_write(portFD, cmd, DOME_CMD, &nbytes_written)) != TTY_OK)
  	{
  		tty_error_msg(rc, errstr, MAXRBUF);
    	if(tty_debug) fprintf(stderr, "%s control_shutter error: %s\n", cmd, errstr);
    	return false;
  	}

  	if(tty_debug) fprintf(stderr, "CMD (%s)\n", cmd);

    usleep(100000);

  	if((rc = tty_read(portFD, resp, DOME_CMD, DOME_TIMEOUT, &nbytes_read)) != TTY_OK)
  	{
  		tty_error_msg(rc, errstr, MAXRBUF);
    	if(tty_debug) fprintf(stderr, "control_shutter error: %s\n",errstr);
    	return false;
  	}

  	resp[nbytes_read] = '\0';

  	if(tty_debug) fprintf(stderr, "RES (%s)\n", resp);

  	if(!strcmp(resp, "d#gotmess"))
  	{
      if(targetShutter == SHUTTER_OPEN)
  		  shutterStatus = SHUTTER_OPENING;
      else if(targetShutter == SHUTTER_CLOSE)
        shutterStatus = SHUTTER_CLOSING;
      else
        shutterStatus = SHUTTER_MOVING;
    	return true;
  	}
  	else
    	return false;
}

const char * dome_get_shutter_status_string(ShutterStatus status)
{
    switch (status)
    {
        case SHUTTER_OPENED:
            return "Shutter is open";
            break;
        case SHUTTER_CLOSED:
            return "Shutter is closed";
            break;
        case SHUTTER_OPENING:
            return "Shutter is opening";
            break;
        case SHUTTER_CLOSING:
            return "Shutter is closing";
            break;
        case SHUTTER_MOVING:
            return "Shutter is in motion";
            break;
        case SHUTTER_UNKNOWN:
            return "Shutter status is unknown";
            break;
    }
}
