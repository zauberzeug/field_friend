#!/usr/bin/env bash

if [ $# -ne 1 ]
then
    echo "Usage:"
    echo "`basename $0` system-name"
    exit
fi

ssh "$1" "cd field_friend; ./docker.sh stop rosys"
rsync -alv "$1/" "$1:.rosys/"
ssh "$1" "cd field_friend; ./docker.sh up rosys"
