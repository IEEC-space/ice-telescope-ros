###############################
## ROS package ice_telescope ##
###############################

cmake_minimum_required(VERSION 2.8.3)
project(ice_telescope)

##########################
## Macros and libraries ##
##########################

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslaunch
)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
set(ice_tel_client_node_HDRS
  include/ice_telescope/TelAppDialog.h
)
qt4_wrap_cpp(ice_tel_client_node_MOCS ${ice_tel_client_node_HDRS})

## Find libusb library
find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

## CFITSIO header(s)
if (NOT CFITSIO_INCLUDES) 
  find_path (CFITSIO_INCLUDES fitsio.h
    PATHS /usr/include /usr/local/include
  )
endif (NOT CFITSIO_INCLUDES)

if (NOT CFITSIO_CFITSIO_LIBRARY)
  find_library (CFITSIO_CFITSIO_LIBRARY cfitsio
    PATHS /usr/lib /usr/local/lib
  )
endif (NOT CFITSIO_CFITSIO_LIBRARY)

## SBIGUDRV 
if (NOT SBIGUDRV_LIBRARY)
	find_library (SBIGUDRV_LIBRARY sbigudrv
		PATHS /usr/lib /usr/local/lib
	)
endif (NOT SBIGUDRV_LIBRARY)

## SNMP
if (NOT NETSNMP_INCLUDES)
  find_path (NETSNMP_INCLUDE net-snmp/net-snmp-includes.h net-snmp/net-snmp-config.h
    PATHS /usr/include /usr/local/include
  )
endif (NOT NETSNMP_INCLUDES)

if (NOT NETSNMP_LIBRARY)
  find_library (NETSNMP_LIBRARY netsnmp libnetsnmp
    PATHS /usr/lib /usr/local/lib
  )
endif (NOT NETSNMP_LIBRARY)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  baader.srv
  sbig.srv
  meade.srv
  apc.srv
  vaisala.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB1_INCLUDE_DIRS}
  ${CFITSIO_INCLUDES}
  ${QT_INCLUDE_DIRS}
  ${NETSNMP_INCLUDES}
)


#################
## Full Server ##
#################

add_executable(ice_tel_server
  src/servers/ice_tel_server.cpp
  src/drivers/tty_com.c
  src/drivers/baader_dome.c 
  src/devices/BaaderDome.cpp
  src/drivers/csbigcam.cpp
  src/drivers/csbigimg.cpp
  src/devices/SbigCcd.cpp
  src/drivers/lx200gps.c
  src/devices/MeadeTelescope.cpp
  src/drivers/snmp_com.c
  src/devices/ApcPdu.cpp
  src/drivers/vaisala_ws.c
  src/devices/VaisalaWS.cpp
)
add_dependencies(ice_tel_server ice_telescope_gencpp)
target_link_libraries(ice_tel_server 
  ${catkin_LIBRARIES} 
  ${LIBUSB_1_LIBRARIES} 
  ${CFITSIO_CFITSIO_LIBRARY} 
  ${SBIGUDRV_LIBRARY}
  ${NETSNMP_LIBRARY}
)

########################
## Baader Dome Client ##
########################

add_executable(baader_client src/clients/baader_client.cpp)
add_dependencies(baader_client ice_telescope_gencpp)
target_link_libraries(baader_client ${catkin_LIBRARIES})

#####################
## SBIG CCD Client ##
#####################

add_executable(sbig_client src/clients/sbig_client.cpp)
add_dependencies(sbig_client ice_telescope_gencpp)
target_link_libraries(sbig_client ${catkin_LIBRARIES})

############################
## Meade Telescope Client ##
############################

add_executable(meade_client src/clients/meade_client.cpp)
add_dependencies(meade_client ice_telescope_gencpp)
target_link_libraries(meade_client ${catkin_LIBRARIES})

################
## APC Client ##
################

add_executable(apc_client src/clients/apc_client.cpp)
add_dependencies(apc_client ice_telescope_gencpp)
target_link_libraries(apc_client ${catkin_LIBRARIES})

####################
## Vaisala Client ##
####################

add_executable(vaisala_client src/clients/vaisala_client.cpp)
add_dependencies(vaisala_client ice_telescope_gencpp)
target_link_libraries(vaisala_client ${catkin_LIBRARIES})
