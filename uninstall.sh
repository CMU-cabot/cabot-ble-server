#!/bin/bash

if [ $(id -u) -eq 0 ]; then
   echo "please do not run as root: $0"
   exit
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

## uninstall cabot-ble-server.service
systemctl --user disable --now cabot-ble-server
INSTALL_DIR=$HOME/.config/systemd/user
rm $INSTALL_DIR/cabot-ble-server.service

## uninstall cabot-ble-server
sudo rm /opt/cabot-ble-server
