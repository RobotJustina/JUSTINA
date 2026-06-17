#!/usr/bin/env bash
set -Eeuo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi

rosdep update --include-eol-distros
rosdep install \
    --from-paths "${JUSTINA_WS}/src" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    --skip-keys "libopenni-sensor-primesense-dev" \
    -r \
    -y

cd "${JUSTINA_WS}"
catkin clean -y
catkin build
