sudo apt-get -y remove docker docker-engine docker.io
sudo apt-get update
sudo apt-get -y install \
	apt-transport-https \
	ca-certificates \
	curl \
	software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo apt-key fingerprint 0EBFCD88

sudo add-apt-repository \
	"deb [arch=amd64] https://download.docker.com/linux/ubuntu \
	$(lsb_release -cs) \
	stable"

sudo apt-get update

sudo apt-get install docker-ce

sudo groupadd docker

sudo usermod -aG docker $USER

docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
sudo apt-get purge -y nvidia-docker


curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
	sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
	sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update

sudo apt-get install -y nvidia-docker2
sudo pkill -SIGHUP dockerd

docker pull rmartella/justina-opencv:latest

docker network create --subnet=172.19.0.0/16 justinaNet

echo "alias justina-opencv='nvidia-docker run -h justina --privileged  --security-opt label=disable  --security-opt seccomp=unconfined  --env="DISPLAY" --env QT_X11_NO_MITSHM=1  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --expose=11311 -e HOST_NAME=$(hostname) -e IP_HOST=172.19.0.2 --net justinaNet --ip 172.19.0.2 --mount src=~/docker_volumen,target=/home/biorobotica/docker_volumen,type=bind --name="justina-opencv" rmartella/justina-opencv:latest terminator'" >> /home/$USER/.bashrc
echo alias justina-yolo='nvidia-docker run -h justina --privileged  --security-opt label=disable  --security-opt seccomp=unconfined  --env="DISPLAY" --env QT_X11_NO_MITSHM=1  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --expose=11311 -e HOST_NAME=$(hostname) -e IP_HOST=172.19.0.2 --net justinaNet --ip 172.19.0.2 --mount src=~/docker_volumen,target=/home/biorobotica/docker_volumen,type=bind --privileged -v /dev/video0:/dev/video0 --name="justina-yolo" justina-yolo:latest terminator' >> /home/$USER/.bashrc
