#!/usr/bin/env bash
micromamba create -n cgn_env -c conda-forge -c robostack-staging catkin_tools ros-noetic-desktop
eval "$(micromamba shell hook --shell bash)"
micromamba activate cgn_env
pip install cgn-pytorch -f https://data.pyg.org/whl/torch-2.1.0+cu121.html
