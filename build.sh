#!/bin/bash

docker compose build --build-arg UID=$UID
docker compose run --rm ble /launch.sh build