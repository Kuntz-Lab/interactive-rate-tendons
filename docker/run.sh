#!/bin/bash

set -e
set -u
set -o pipefail
set -x

local_port=3322

# Per advice from:
#
#   https://github.com/tonyOreglia/argument-counter/wiki/How-to-use-GDB-within-Docker-Container
#
# We can add "--cap-add=SYS_PTRACE" and "--security-opt seccomp=unconfined"
# to allow for gdb support from within the docker container

# TODO: add logic to check if it already exists and if it's already running
docker run \
  --privileged \
  -v /dev:/dev \
  -v ${HOME}:${HOME} \
  --name tendon-dev \
  --detach \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  --publish ${local_port}:22 \
  tendon-dev

set +x
echo
echo "Now you may connect to the container.  For example, with:"
echo "    ssh -f -p ${local_port} localhost DISPLAY=\${DISPLAY} terminator"
echo "or simply"
echo "    ssh -p ${local_port} localhost"
echo
echo "when you're done, do"
echo "    docker stop ros2-dev"
echo "to start again, call"
echo "    docker start ros2-dev"
echo "then connect with ssh."
echo

unset local_port
