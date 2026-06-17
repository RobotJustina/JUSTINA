#!/usr/bin/env bash
set -Eeuo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${JUSTINA_WS}/devel/setup.bash" ]]; then
    source "${JUSTINA_WS}/devel/setup.bash"
fi

exec "$@"
