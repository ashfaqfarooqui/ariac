#!/bin/bash

# generate_video.sh: A shell script to generate a video from a Gazebo log file.
#
# E.g.: ./generate_video.sh ~/ariac_qual3/qual3b/log/gazebo/state.log output.ogv
#
# Please, install the following dependencies before using the script:
#   sudo apt-get install recordmydesktop wmctrl

set -e

# Constants
BLACK_WINDOW_TIME=30
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

# Define usage function.
usage()
{
  echo "Usage: $0 <path_to_gazebo_log_file> <output>"
  exit 1
}

# Wait until the /gazebo/ariac/world_stats topic tells us that the playback
# has been paused. This event will trigger the end of the recording.
wait_until_playback_ends()
{
  echo -n "Waiting for playback to end..."
  until gz topic -e /gazebo/ariac/world_stats -d 1 -u | grep "paused: true" \
    > /dev/null
  do
    sleep 1
  done
  echo -e "${GREEN}OK${NOCOLOR}"
}

# Call usage() function if arguments not supplied.
[[ $# -ne 2 ]] && usage

GZ_LOG_FILE=$1
OUTPUT=$2

# Start Gazebo in playback mode (paused).
roslaunch osrf_gear gear_playback.launch state_log_path:=$GZ_LOG_FILE \
  > /dev/null 2>&1 &

# Wait and find the Gazebo Window ID.
until wmctrl -lp | grep Gazebo > /dev/null
do
  sleep 1
done
GAZEBO_WINDOW_ID=`wmctrl -lp | grep Gazebo | cut -d" " -f 1`

if [ -z "$GAZEBO_WINDOW_ID" ]; then
  echo "Gazebo window not detected. Exiting..."
  sleep 2
  killall -w gzserver gzclient
  exit 1
fi

# Adjust the value of this constant if needed to avoid capturing a black
# screen for a long time.
sleep $BLACK_WINDOW_TIME

# Play the simulation.
echo -n "Playing back..."
gz world -p 0
echo -e "${GREEN}OK${NOCOLOR}"

# Start recording the Gazebo Window.
echo -n "Recording..."
recordmydesktop --windowid=$GAZEBO_WINDOW_ID -o $OUTPUT 2> /dev/null &
echo -e "${GREEN}OK${NOCOLOR}"

# Wait until the playback ends.
wait_until_playback_ends

# Terminate Gazebo.
killall -w gzserver gzclient recordmydesktop
