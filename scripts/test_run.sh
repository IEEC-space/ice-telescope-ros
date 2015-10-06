#!/bin/bash

echo "Starting observation"

echo "Initializing roscore"
if !(ps aux | grep -q '[r]oscore'); then
  roscore &
fi

echo "Initializing servers"
if !(ps aux | grep -q '[b]aader_server'); then
  rosrun ice_telescope baader_server &
fi

if !(ps aux | grep -q '[s]big_server'); then  
  rosrun ice_telescope sbig_server &
fi

if !(ps aux | grep -q '[m]eade_server'); then
  rosrun ice_telescope meade_server &
fi

sleep 2

if !(rosrun ice_telescope baader_client status | grep open); then
  if (rosrun ice_telescope baader_client open); then
    echo "Opening dome"
    sleep 60 
  else
    echo "Error opening dome"
  fi
fi


echo "Doing things..."
sleep 5


if (rosrun ice_telescope baader_client close); then
  echo "Closing dome"
  sleep 60
else
  echo "Error closing dome"
fi  

echo "Finished observation"
