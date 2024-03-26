#!/bin/bash

# NOTE: This needs to be run again for non-interactive sessions
source /opt/ros/noetic/setup.bash

# Fix for when the volume is mounted and it doesn't exist
sudo chown -R ros:ros /home/ros/catkin_ws/src

# If there are commands passed to the script, execute them
if [ $# -gt 0 ]; then
    eval "$@"
    # catkin_make
    # source devel/setup.bash
else
    # Default behavior or message if no commands are passed
    echo "No command specified, starting a bash shell."
    exec /bin/bash
fi
