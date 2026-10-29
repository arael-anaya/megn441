#! /bin/bash

# Be sure that docker container has been built first:
# docker build -t ros2-dev .

# Note: Change the location of your linked folder
# if it is anything other than ~/megn441.
# $(id -un) just plugs in your username from linux
# /home/$(id -un) is the same as ~

LOCAL_DIR="/home/$(id -un)/megn441"

# This folder maps to /mnt/docker_ws within the docker container


docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${LOCAL_DIR}:/mnt/docker_ws \
  -w /mnt/docker_ws \
  -e DISPLAY \
  -e WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR \
  -e PULSE_SERVER \
  ros2-dev:latest
