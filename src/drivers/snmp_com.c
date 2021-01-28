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

#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "ice_telescope/snmp_com.h"

netsnmp_session session, *ss;

bool snmpc_open(const char *host) {
  init_snmp("pdudrv");
  snmp_sess_init(&session);
  session.peername=strdup(host);
  session.version=SNMP_VERSION_2c;
  session.community="private";
  session.community_len=strlen(session.community);
  SOCK_STARTUP;
  ss=snmp_open(&session);
  if(!ss)
    return false;
  return true;
}

bool snmpc_check_session()
{
  if(!ss)
    return false;
  return true;
}

int snmpc_read(const char *oid2,char *buffer,int len) {
  netsnmp_pdu *pdu;
  netsnmp_pdu *response;
  oid anOID[MAX_OID_LEN];
  size_t anOID_len;
  netsnmp_variable_list *vars;
  int status;
  int i;
  
  for(i=0;i<len;i++) buffer[i]=0;
  pdu = snmp_pdu_create(SNMP_MSG_GET);
  anOID_len = MAX_OID_LEN;
  if(!snmp_parse_oid(oid2, anOID, &anOID_len)) {
    SOCK_CLEANUP;
    return 0;
  }
  snmp_add_null_var(pdu, anOID, anOID_len);
  status = snmp_synch_response(ss, pdu, &response);
  if(status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR) {
    for(vars = response->variables; vars; vars = vars->next_variable) {
      //print_variable(vars->name, vars->name_length, vars);
      if (vars->type == ASN_INTEGER) {
        if(vars->val_len<len) {
          if(*(vars->val.integer) == 1)
            sprintf(buffer, "ON");
          else
            sprintf(buffer, "OFF");
        }
      }
    }
  } else {
    if(status == STAT_SUCCESS) fprintf(stderr, "Error in packet: Reason: %s\n",snmp_errstring(response->errstat));
    else if(status == STAT_TIMEOUT) fprintf(stderr, "Timeout: No response from %s.\n",session.peername);
    else fprintf(stderr,"Unknown error\n");
    return 0;
  }
  if(response) snmp_free_pdu(response);
  if(!ss) return 0;
  if(buffer[0]==0) return 0;
  return 1;
}

int snmpc_walk(const char *oid2,char *buffer,int len) {
  netsnmp_pdu *pdu;
  netsnmp_pdu *response;
  oid anOID[MAX_OID_LEN];
  size_t anOID_len;
  netsnmp_variable_list *vars;
  int status;
  int i;
  
  for(i=0;i<len;i++) buffer[i]=0;
  pdu = snmp_pdu_create(SNMP_MSG_GETBULK);
  pdu->non_repeaters = 0; 
  pdu->max_repetitions = 8;
  anOID_len = MAX_OID_LEN;
  if(!snmp_parse_oid(oid2, anOID, &anOID_len)) {
    SOCK_CLEANUP;
    return 0;
  }
  snmp_add_null_var(pdu, anOID, anOID_len);
  status = snmp_synch_response(ss, pdu, &response);
  if(status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR) {
    for(vars = response->variables; vars; vars = vars->next_variable) {
      //print_variable(vars->name, vars->name_length, vars);
      if (vars->type == ASN_INTEGER) {
        if(vars->val_len<len) {
          if(*(vars->val.integer) == 1)
            sprintf(buffer, "%s ON", buffer);
          else
            sprintf(buffer, "%s OFF", buffer);
        }
      }
    }
  } else {
    if(status == STAT_SUCCESS) fprintf(stderr, "Error in packet: Reason: %s\n",snmp_errstring(response->errstat));
    else if(status == STAT_TIMEOUT) fprintf(stderr, "Timeout: No response from %s.\n",session.peername);
    else fprintf(stderr,"Unknown error\n");
    return 0;
  }
  if(response) snmp_free_pdu(response);
  if(!ss) return 0;
  if(buffer[0]==0) return 0;
  return 1;
}

int snmpc_write(const char *oid2, char *buffer, int len, char type, const char *val) {
  netsnmp_pdu *pdu;
  netsnmp_pdu *response;
  oid anOID[MAX_OID_LEN];
  size_t anOID_len;
  netsnmp_variable_list *vars;
  int status;
  int i;
  
  for(i=0;i<len;i++) buffer[i]=0;
  pdu = snmp_pdu_create(SNMP_MSG_SET);
  anOID_len = MAX_OID_LEN;
  if(!snmp_parse_oid(oid2, anOID, &anOID_len)) {
    SOCK_CLEANUP;
    return 0;
  }
  snmp_add_var(pdu, anOID, anOID_len, type, val);
  status = snmp_synch_response(ss, pdu, &response);
  if(status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR) {
    for(vars = response->variables; vars; vars = vars->next_variable) {
      //print_variable(vars->name, vars->name_length, vars);
      if (vars->type == ASN_INTEGER) {
        if(vars->val_len<len) {
          if(*(vars->val.integer) == 1)
            sprintf(buffer, "ON");
          else
            sprintf(buffer, "OFF");
        }
      }
    }
  } else {
    if(status == STAT_SUCCESS) fprintf(stderr, "Error in packet: Reason: %s\n",snmp_errstring(response->errstat));
    else if(status == STAT_TIMEOUT) fprintf(stderr, "Timeout: No response from %s.\n",session.peername);
    else fprintf(stderr,"Unknown error\n");
    return 0;
  }
  if(response) snmp_free_pdu(response);
  if(!ss) return 0;
  if(buffer[0]==0) return 0;
  return 1;
}

void snmpc_close(void) {
  snmp_close(ss);
}
