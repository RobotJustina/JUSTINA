#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> $HOME/.bashrc
echo "export ROS_MASTER_URI=http://$HOST_NAME:11311/" >> $HOME/.bashrc
source ~/.bashrc
if [ ! -z ${HOST_IP+x} ]; then
	echo 'biorobotica' | sudo -S -- sh -c "echo \"$HOST_IP   $HOST_NAME\" >> /etc/hosts";
fi
echo -e "\n"
#echo 'test' | sudo tee -a /etc/hosts
exec "$@"
