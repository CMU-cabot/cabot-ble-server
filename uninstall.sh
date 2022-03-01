#!/bin/bash

if [ $(id -u) -ne 0 ]; then
   echo "please run as root: sudo $0"
   exit
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

## uninstall cabot-ble-server.service
systemctl disable --now cabot-ble-server
rm /etc/systemd/system/cabot-ble-server.service

## uninstall cabot-ble-server
INSTALL_DIR=/opt/cabot-ble-server
INSTALL_FILE="Dockerfile docker-compose.yaml cabot_ble.py cabot cabot_ui requirements.txt"

rm -rf $INSTALL_DIR
