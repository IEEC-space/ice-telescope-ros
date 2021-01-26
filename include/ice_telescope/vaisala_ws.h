/*
    ICE Telescope ROS Package: Vaisala Weather Station driver
    Copyright (C) IEEC 2021
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

#ifndef VaisalaWS_H
#define VaisalaWS_H

bool ws_connect(int *in_fd);
bool ws_disconnect(int portFD);
bool ws_ack(int portFD);
bool ws_getinfo(int portFD, char *info);
bool ws_reset(int portFD);

#endif