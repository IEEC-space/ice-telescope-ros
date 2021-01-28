/*
    ICE Telescope ROS Package: SNMP common routines
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

#ifndef SNMPCOM_H
#define SNMPCOM_H

bool snmpc_open(const char *host);
bool snmpc_check_session();
int snmpc_read(const char *oid2,char *buffer,int len);
int snmpc_write(const char *oid2, char *buffer, int len, char type, const char *val);
int snmpc_walk(const char *oid2,char *buffer,int len);
void snmpc_close(void);

#endif