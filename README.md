ROS-TELESCOPE: ICE Telescope - A ROS package
================

`ice_telescope` is a ROS package to operate and remote control the
telescope system at the ICE building in the UAB Campus. The full system
is composed of a Meade LX200GPS telescope, a SBIG ST-7 CCD camera, a
Baader Planetarium dome, an APC Switched PDU and a Vaisala weather station.
The work shown here was originally published in [Vilardell et al., "Using Robotic Operating System (ROS) to control autonomous observatories", Proc. SPIE 9913, Software and Cyberinfrastructure for Astronomy IV, 99132V (26 July 2016)](https://doi.org/10.1117/12.2232694).

[![Using ROS to control an observatory demo](https://user-images.githubusercontent.com/22243223/149656654-004ae4ca-e41e-4bab-8fbd-a0588bc954c3.png)](https://youtu.be/R92qWKE9lYU)

Demo at: https://youtu.be/R92qWKE9lYU

### Table of Contents

-   [Synopsis](#synopsis)
-   [Description](#description)
-   [Servers](#servers)
-   [Telescope client](#telescope-client)
-   [CCD client](#ccd-client)
-   [Dome client](#dome-client)
-   [PDU client](#pdu-client)
-   [Weather Station client](#ws-client)
-   [Example](#example)
-   [See Also](#see-also)
-   [Requirements](#requirements)
-   [Changes](#changes)
-   [Version](#version)
-   [License and Copyright](#license-and-copyright)
-   [Authors](#authors)

Synopsis
--------

**ROS**

    roscore

**Server**

    rosrun ice_telescope ice_telescope_node

**Client**

    rosrun ice_telescope ice_telescope_node action [params]

Description
-----------

`ice_telescope` is composed of several nodes --`ice_telescope_node`--
that allow the control of the telescope system. In addition to the 
server to control all devices `ice_tel_server`, each of the system
components (telescope, dome, ccd, pdu, weather station) has a pair of client--server nodes following the naming convention `brand_server` and `brand_client`:

**Full-Server**:   `ice_tel_server`.

**Telescope**:   `meade_server` and `meade_client`.

**CCD**:   `sbig_server` and `sbig_client`.

**Dome**:   `baader_server` and `baader_client`.

**PDU**:    `apc_server` and `apc_client`.

**WS**:     `vaisala_server` and `vaisala_client`.

**Note:** The `brand_server` servers are there for your convenience but only the `ice_tel_server` is necessary to control them all.

The server node runs continuously waiting for petitions from the client
node. When a client node's petition is received by the server node, the
server processes the petition, sends a response back to the client and
returns to the waiting mode. The client waits for the server response
and finishes the execution.

**Client node**

The **action** parameter issues the desired order to the server.

\[**params**\] will depend on the system component and the selected
**action**.

**Note:** `roscore` must be running at all times for node communication
and interoperation.

Servers
-------

The servers for all the system elements are executed without additional
parameters and they must be running to listen to the clients commands.

**Full-Server**

    rosrun ice_telescope ice_tel_server

**Telescope**

    rosrun ice_telescope meade_server
  
**CCD**
  
    rosrun ice_telescope sbig_server
   
**Dome**
  
    rosrun ice_telescope baader_server

**PDU**

    rosrun ice_telescope apc_server

**WS**

    rosrun ice_telescope vaisala_server

Telescope client
----------------

The telescope client issues the user's desired actions to perform with
the Meade LX200GPS Telescope.

    rosrun ice_telescope meade_client action [params]

**Note:** To run more than one meade_client node at the same time it is necessary to specify a name for the node in the above commands as follows [__name:=DesiredName]

### Options

The **action** parameter is the command to be sent to the server. The
**action** can be one of the following:

**init**

Initialize the telescope for a remote session.

    rosrun ice_telescope meade_client init

**goto** 

Point the telescope to the specified coordinates. 

    rosrun ice_telescope meade_client goto ra dec

        -   ra: Right ascension as a double value.
        -   dec: Declination as a double value.

**messier**, **star**, **deepsky**

Point the telescope to the selected catalog object.

    rosrun ice_telescope meade_client messier objectNum
    rosrun ice_telescope meade_client star objectNum
    rosrun ice_telescope meade_client deepsky objectNum

        -   objectNum: The catalog number for the desired object.

**move**

Move the telescope in a specific direction for a specific period of time.

    rosrun ice_telescope meade_client move dir(north/south/east/west) milliseconds

        -   dir: the desired movement direction. The possible directions are: north, south, east, west.
        -   milliseconds: the duration of the movement as a four-digit number (0-9999).

**sync**

Synchronize the telescope coordinates with the current ones.

    rosrun ice_telescope meade_client sync
    rosrun ice_telescope meade_client sync ra dec

        -   ra: Right ascension as a double value.
        -   dec: Declination as a double value.

**park**

Slew the telescope to the parked position. **Note:** after parking, a power cycle is required.

    rosrun ice_telescope meade_client park

**status**

Check if the telescope is moving or IDLE.

    rosrun ice_telescope meade_client status

**gps** 

Update the system's gps. **Note:** The dome must be open for the
gps sync.

    rosrun ice_telescope meade_client gps

**getobjradec**

Get the coordinates of the currently selected object.

    rosrun ice_telescope meade_client getobjradec

**gettelradec**

Get the telescope's current pointing coordinates.

    rosrun ice_telescope meade_client gettelradec

**getdatetime**

Get the telescope's current date and time.

    rosrun ice_telescope meade_client getdatetime

**setdatetime**

Set the telescope's date and time to the current ones.

    rosrun ice_telescope meade_client setdatetime

**getlatlon**

Get the telescope's latitude and longitude.

    rosrun ice_telescope meade_client getlatlon

**setlatlon**

Set the telescope's latitude and longitude.

    rosrun ice_telescope meade_client setlatlon lat lon

        -   lat: The current latitude as a double value.
        -   lon: The current longitude as a double value.

**focus**

Move the telescope's focus (in/out). WORK IN PROGRESS.

**reconnect**

Re-establish telescope connection.

    rosrun ice_telescope meade_client reconnect


CCD client
----------

The CCD client issues the user's desired actions to perform with the
SBIG ST-7 CCD.

    rosrun ice_telescope sbig_client action [params]

**Note:** To run more than one sbig_client node at the same time it is necessary to specify a name for the node in the above commands as follows [__name:=DesiredName]

### Options

The **action** parameter is the command to be sent to the server. The
**action** can be one of the following:

**capture**

Start an exposure and save the result to file.

    rosrun ice_telescope sbig_client capture filePath fileType imgCount imgType expTime readoutMode top left width height fastReadout dualReadoutChannel

        -   filePath: The path for the saved image files.
        -   fileType: FITS or SBIG file formats.
        -   imgCount: Number of exposures to take.
        -   imgType: LF (light frame) or DF (dark frame).
        -   expTime: Number of seconds (or fraction of second) of exposure.
        -   readoutMode: Binning. Options: 1x1, 2x2, 3x3.
        -   top: Starting position in the 'Y' axis.
        -   left: Starting position in the 'X' axis.
        -   width: Image width in pixels.
        -   height: Image height in pixels.

        **Note:** If all params (top, left, width and height) are zero,
        the full size of the CCD image is used.

        -   fastReadout: 1 for fast readout and 0 for normal readout.
        -   dualReadoutChannel: 1 for dual channel readout an 0 for
        single channel readout.

**settemp**

Enable or disable the cooler to achieve the desired temperature for the CCD.

    rosrun ice_telescope sbig_client settemp enable temperature

        -   enable: 1 to enable and 0 to disable.
        -   temperature: double value with the desired temperature

**gettemp**

Query the CCD temperature. The server returns the temperature, the
power applied to the CCD as a percentage (0-1) and the cooler
status (enabled/disabled).

    rosrun ice_telescope sbig_client gettemp

**getcapstatus**

Query the CCD capture status. The server returns the exposure progress percentage or the IDLE status.

    rosrun ice_telescope sbig_client getcapstatus

**reconnect**

Re-establish CCD connection.

    rosrun ice_telescope sbig_client reconnect


Dome client
-----------

The dome client issues the user's desired actions to perform with the
Baader Planetarium Dome.

    rosrun ice_telescope baader_client action

**Note:** To run more than one baader_client node at the same time it is necessary to specify a name for the node in the above commands as follows [__name:=DesiredName]

### Options

The **action** parameter is the command to be sent to the server. The
**action** can be one of the following:

**open**

Open the dome.

    rosrun ice_telescope baader_client open

**close**

Close the dome.

    rosrun ice_telescope baader_client close

**status**

Query the dome status. The possible states for the dome are: open,
closed, moving and unknown.

    rosrun ice_telescope baader_client status

**reconnect**

Re-establish dome connection.

    rosrun ice_telescope baader_client reconnect


PDU client
----------

The PDU client issues the user's desired actions to perform with the APC Switched PDU.

    rosrun ice_telescope apc_client action device

**Note:** To run more than one apc_client node at the same time it is necessary to specify a name for the node in the above commands as follows [__name:=DesiredName]

### Options

The **action** parameter is the command to be sent to the server and the **device** parameter specifies on which system element the action has to be performed. The **device** parameter can be one of the following:

**telescope**

The specified **action** will be performed on the telescope.

    rosrun ice_telescope apc_client [action] telescope

**ccd**

The specified **action** will be performed on the CCD.

    rosrun ice_telescope apc_client [action] ccd

**vaisala**

The specified **action** will be performed on the weather station.

    rosrun ice_telescope apc_client [action] vaisala

**light**

The specified **action** will be performed on the light inside the dome.

    rosrun ice_telescope apc_client [action] light


The **action** can be one of the following:

**power_on**

Power on the specified device by switching on the corresponding outlet of the PDU.

    rosrun ice_telescope apc_client power_on [device]

**power_off**

Power off the specified device by switching off the corresponding outlet of the PDU.

    rosrun ice_telescope apc_client power_off [device]

**power_status**

Check the corresponding PDU's outlet status.

    rosrun ice_telescope apc_client power_status [device]


Weather Station client
----------------------

The weather station client issues the user's desired actions to perform with the Vaisala weather station. The only action for the weather station is **getinfo**.

    rosrun ice_telescope vaisala_client getinfo

**Note:** To run more than one vaisala_client node at the same time it is necessary to specify a name for the node in the above commands as follows [__name:=DesiredName]


Example
-------

```bash
$ roscore &

$ rosrun ice_telescope ice_tel_server &

$ rosrun ice_telescope baader_client open
$ rosrun ice_telescope sbig_client settemp 1 10.0
$ rosrun ice_telescope meade_client gps
$ rosrun ice_telescope meade_client setdatetime
$ rosrun ice_telescope meade_client messier 31

$ rosrun ice_telescope sbig_client capture /img/ FITS 10 LF 30.0 1x1 0 0 0 0 1 1
$ rosrun ice_telescope baader_client close
```

See Also
--------

ROS, `rosrun`, `roscd`, `rosls`, `catkin_make`.

Requirements
------------

**ROS Environment:**   `ice_telescope` requires ROS version &gt;= 1.11.16 (&gt;=
    Indigo distribution).

**ROS Workspace:**   If you want to compile or install the distributed system, you need a
    `catkin` workspace.

**Libraries:** 

  - libusb-1.0
  - cfitsio
  - libsnmp-dev
  - libsbigudrv. This library can be downloaded from [here](http://archive.sbig.com/sbwhtmls/devswframe.htm).

Changes
-------

Please check the file [`CHANGELOG`](CHANGELOG) for the list of changes and
acknowledgment to people contributing bugfixes or enhancements.

Version
-------

Version: 0.1.3 of 2016/02/17.

License and Copyright
---------------------

    ICE Telescope ROS Package
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

Authors
------

Biel Artigues Aguilo, Francesc Vilardell Sall??s

IEEC website: http://www.ieec.cat/

ICE telescope website: https://www.ice.csic.es/technology/labs
