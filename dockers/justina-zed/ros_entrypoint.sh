#!/bin/bash
set -e

# setup ros environment

echo "export ROS_MASTER_URI=http://$HOST_NAME:11311/" >> $HOME/.bashrc
source ~/.bashrc
if [ ! -z ${HOST_IP+x} ]; then
	echo -S -- sh -c "echo \"$HOST_IP   $HOST_NAME\" >> /etc/hosts";
fi
echo -e "\n"
#echo 'test' | sudo tee -a /etc/hosts
exec "$@"
