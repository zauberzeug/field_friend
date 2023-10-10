#!/usr/bin/env bash

if [ $# -ne 1 ]
then
    echo "Usage:"
    echo "`basename $0` system-name"
    exit
fi

rsync -aluv "$1:.rosys/*json" $1/
