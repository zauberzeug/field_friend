#!/usr/bin/env bash

if [ $# -ne 1 ]
then
    echo "Usage:"
    echo "`basename $0` <bin>"
    exit
fi
bin=$1

id=`dfu-util --list | sed -n 's/.*serial="\(.*\)"/\1/p' | tail -n 1`
if [ -z $id ]
then
    echo "Could not find ODrive."
    echo "Please connect ODrive and turn it on in DFU mode."
    exit
fi

echo "Flashing $bin onto ODrive with ID $id..."
sudo dfu-util -S $id -a 0 -s 0x08000000 -D "$bin"
