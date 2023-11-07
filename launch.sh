#!/bin/bash

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

log_prefix=cabot-ble-server

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`

NO_BUILD=false

source $scriptdir/.env

if $NO_BUILD; then
    docker-compose up --no-build ble | tee log/$log_name.log 1>&2
else
    docker-compose build --build-arg UID=$UID && \
    docker-compose up ble | tee log/$log_name.log 1>&2
fi

