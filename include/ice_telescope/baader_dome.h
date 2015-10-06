/*******************************************************************************
 Copyright(c) 2014 Jasem Mutlaq. All rights reserved.

 Baader Planetarium Dome INDI Driver

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

#ifndef BaaderDome_H
#define BaaderDome_H

#include <stdbool.h>

typedef enum { DOME_UNKNOWN, DOME_READY } DomeStatus;
typedef enum { SHUTTER_OPENED,  SHUTTER_CLOSED,  SHUTTER_MOVING, SHUTTER_UNKNOWN } ShutterStatus;
typedef enum { SHUTTER_OPEN, SHUTTER_CLOSE } ShutterOperation;

DomeStatus domeStatus;
ShutterStatus shutterStatus;
ShutterOperation targetShutter;
int portFD;

void dome_init_params();
bool dome_connect();
bool dome_disconnect();
bool dome_ack();
bool dome_shutter_status();
bool dome_control_shutter(ShutterOperation operation);
const char * dome_get_shutter_status_string(ShutterStatus status);

#endif