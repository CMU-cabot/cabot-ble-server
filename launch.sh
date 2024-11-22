#!/bin/bash

stop_launch() {
    docker compose -f $dcfile down
    exit 0
}

trap 'stop_launch' SIGINT SIGTERM

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
prefix=$(basename `pwd`)

function help {
    echo "Usage: $0"
    echo ""
    echo "-h         show this help"
    echo "-d         development launch"
}

development=0
dcfile=docker-compose-prodimg.yaml

while getopts "hd" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    d)
        development=1
        dcfile=docker-compose.yaml
        ;;
    esac
done

log_prefix=cabot-app-server
log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`

source $scriptdir/.env

if [[ $development -eq 1 ]]; then
    echo "This is development environment, building the workspace"
    com="docker compose -f $dcfile run --rm cabot-app-server /launch.sh build"
    echo $com
    eval $com
fi

com="docker compose -f $dcfile up 2>&1 | tee log/$log_name.log"
echo $com
eval $com
