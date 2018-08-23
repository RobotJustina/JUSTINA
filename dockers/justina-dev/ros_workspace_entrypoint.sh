#!/bin/bash
set -e

# setup workspace environment
if [ ! -z ${ROS_WORKSPACE+x} ]; then
	echo "source \"${ROS_WORKSPACE}/devel/setup.bash\"" >> $HOME/.bashrc
fi
source ~/.bashrc
echo -e "\n"
#echo 'test' | sudo tee -a /etc/hosts
exec "$@"
