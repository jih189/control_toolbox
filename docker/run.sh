#!/bin/bash
set -ex
docker run -v $PWD/../:/root/planner \
    -e DISPLAY=":0" \
	-e QT_X11_NO_MITSHM=1 \
	-e XAUTHORITY \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	--ipc=host \
	--gpus all \
	--network="host" \
	-p 8888:8888 \
	-p 6006:6006 \
    -p 23000-23500:23000-23500 \
	--privileged=true \
	-v /etc/localtime:/etc/localtime:ro \
	-v /dev/video0:/dev/video0 \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -p 19997:19997 --rm -it coppeliasim-ubuntu22 bash