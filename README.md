ROS-TELESCOPE: ICE Telescope - A ROS package
================

`ice_telescope` is a ROS package to operate and remote control the
telescope system at the ICE building in the UAB Campus. The full system
is composed of a Meade LX200GPS telescope, an SBIG ST-7 CCD camera and a
Baader Planetarium dome.

### Table of Contents

-   [Synopsis](#synopsis)
-   [Description](#description)
-   [Servers](#servers)
-   [Telescope client](#telescope-client)
-   [CCD client](#ccd-client)
-   [Dome client](#dome-client)
-   [Files](#files)
-   [Example](#example)
-   [See Also](#see-also)
-   [Requirements](#requirements)
-   [Changes](#changes)
-   [Version](#version)
-   [License and Copyright](#license-and-copyright)
-   [Author](#author)

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
components (telescope, dome, ccd) has a pair of client--server nodes
following the naming convention `brand_server` and `brand_client`:

**Full-Server**:   `ice_tel_server`.

**Telescope**:   `meade_server` and `meade_client`.

**CCD**:   `sbig_server` and `sbig_client`.

**Dome**:   `baader_server` and `baader_client`.

**Note:** The `brand_server` servers are there for your convenience 
but only the `ice_tel_server` is necessary to control them all.

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

Telescope client
----------------

The telescope client issues the user's desired actions to perform with
the Meade LX200GPS Telescope.

    rosrun ice_telescope meade_client action [params]
    rosrun ice_telescope meade_client.py action [params]

    **Note:** To run more than one meade_client node at the same time it is necessary to specify a name for the node in the above commands as follows [__name:=DesiredName]

### Options

The **action** parameter is the command to be sent to the server. The
**action** can be one of the following:

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
    rosrun ice_telescope sbig_client.py action [params]

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
    rosrun ice_telescope baader_client.py action

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

Files
-----

`ice_tel_server` C++ implementation of the server to control all devices.

`meade_server` C++ implementation of the telescope server.

`meade_client` C++ implementation of the telescope client.

`meade_client.py` Python implementation for the telescope client.

`sbig_server` C++ implementation of the CCD server.

`sbig_client` C++ implementation of the CCD client.

`sbig_client.py` Python implementation for the CCD client.

`baader_server` C++ implementation of the dome server.

`baader_client` C++ implementation of the dome client.

`baader_client.py` Python implementation for the dome client.

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

**ROS Environment:**   `ice_telescope` requires ROS version &gt;= 1.11.13 (&gt;=
    Indigo distribution).

**ROS Workspace:**   If you want to compile or install the distributed system, you need a
    `catkin` workspace.

**Libraries:** 

  - libusb-1.0
  - cfitsio
  - libsbigudrv. This library can be downloaded from [here](http://archive.sbig.com/sbwhtmls/devswframe.htm).

Changes
-------

Please check the file [`CHANGELOG`](CHANGELOG) for the list of changes and
acknowledgment to people contributing bugfixes or enhancements.

Version
-------

Version: 0.1.1 of 2015/10/29.

The actual version of `ice_telescope` may be found on the following link:
[`ice_telescope.zip`](https://baiels.redkaos.org/index.php/s/H4i9a87jyLQ4BMc).

License and Copyright
---------------------

    ICE Telescope ROS Package
    Copyright (C) 2015 Biel Artigues Aguilo <artigues@ice.cat>

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

Author
------

Biel Artigues Aguilo

Email: [`artigues@ice.cat`](mailto:artigues@ice.cat)

Web: [`http://www.ice.csic.es/`](http://www.ice.csic.es/)