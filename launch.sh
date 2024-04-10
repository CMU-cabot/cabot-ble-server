#!/bin/bash

stop_launch() {
    docker compose down ble
    exit 0
}

trap 'stop_launch' SIGINT SIGTERM EXIT

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

log_prefix=cabot-ble-server

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`

NO_BUILD=false

source $scriptdir/.env

if $NO_BUILD; then
    docker compose up --no-build ble 2>&1 | tee log/$log_name.log
else
    docker compose build --build-arg UID=$UID && \
    docker compose up ble 2>&1 | tee log/$log_name.log
fi

