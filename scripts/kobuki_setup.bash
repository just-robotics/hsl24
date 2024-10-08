#!/bin/bash

KOBUKI_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

sudo apt-get install -y vim mc ssh

sudo bash $KOBUKI_ROOT_DIR/scripts/udev_rules/create_udev.bash

echo " " >> ~/.bashrc
echo "alias kobuki_docker_into='bash $KOBUKI_ROOT_DIR/docker/into.bash'" >> ~/.bashrc
echo "alias kobuki_docker_run='bash  $KOBUKI_ROOT_DIR/docker/run.bash'" >> ~/.bashrc
