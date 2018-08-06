#!/bin/bash
set -e

alias ll='ls $LS_OPTIONS -l'

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
