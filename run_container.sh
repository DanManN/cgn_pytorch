#!/usr/bin/env bash
xhost +
docker run --gpus all --rm -it -e DISPLAY=${DISPLAY} -v /tmp:/tmp cgn_ros bash
