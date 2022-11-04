#!/bin/bash

log_prefix=cabot-ble-server

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`

docker-compose build --build-arg UID=$UID && \
docker-compose up ble | tee log/$log_name.log 1>&2

