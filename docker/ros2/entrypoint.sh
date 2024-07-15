#!/bin/bash

set -e

export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
source install/setup.bash

exec "$@"
