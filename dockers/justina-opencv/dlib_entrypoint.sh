#!/bin/bash
set -e

#setup dlib python environment
echo "export PYTHONPATH=$PYTHONPATH:$HOME/docker_volumen/facenet" >> $HOME/.bashrc
echo "export PYTHONPATH=$PYTHONPATH:$HOME/docker_volumen/facenet/src" >> $HOME/.bashrc
exec "$@"
