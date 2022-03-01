#!/bin/bash

if [ $(id -u) -ne 0 ]; then
   echo "please run as root: sudo $0"
   exit
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

## install cabot-ble-server
INSTALL_DIR=/opt/cabot-ble-server
INSTALL_FILE="Dockerfile docker-compose.yaml cabot_ble.py cabot cabot_ui requirements.txt"

mkdir -p $INSTALL_DIR
cp -r $INSTALL_FILE $INSTALL_DIR
cd $INSTALL_DIR
docker-compose build


## install cabot-ble-server.service
cd $scriptdir
cp cabot-ble-server.service /etc/systemd/system
systemctl daemon-reload
systemctl enable --now cabot-ble-server
