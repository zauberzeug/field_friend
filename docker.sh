#!/usr/bin/env bash

if [ $# -eq 0 ]
then
    echo "Usage:"
    echo
    echo "  `basename $0` (u | up)      [<containers>]      Create and start"
    echo "  `basename $0` (p | pull)    [<containers>]      pull containers"
    echo "  `basename $0` (U | uppull)  [<containers>]      Create and start (force pull)"
    echo "  `basename $0` (d | down)    [<containers>]      Stop and remove"
    echo "  `basename $0` (s | start)   [<containers>]      Start"
    echo "  `basename $0` (r | restart) [<containers>]      Restart"
    echo "  `basename $0` (h | stop)    [<containers>]      Stop (halt)"
    echo "  `basename $0` (i | install) [<containers>]      Autorestart running containers on failure/reboot"
    echo "  `basename $0` ps            [<containers>]      List"
    echo "  `basename $0` rm            [<containers>]      Remove"
    echo "  `basename $0` stats                             Show statistics"
    echo
    echo "  `basename $0` (l | log)    <container>            Show log tail (last 100 lines)"
    echo "  `basename $0` (e | exec)   <container> <command>  Execute command"
    echo "  `basename $0` (a | attach) <container>            Attach to container with shell"
    echo
    echo "  `basename $0` prune      Remove all unused containers, networks and images"
    echo "  `basename $0` stopall    Stop all running containers (system-wide!)"
    echo "  `basename $0` killall    Kill all running containers (system-wide!)"
    echo
    echo "Arguments:"
    echo
    echo "  containers    One or more containers (omit to affect all containers)"
    echo "  container     Excactly one container to be affected"
    echo "  command       Command to be executed inside a container"
    exit
fi

os=`uname`
case $os in
    Linux)
        # Check if on Jetson platform
        if [ -f /etc/nv_tegra_release ]; then
            # Extract the R number
            r_number=$(grep -oP 'R\d+' /etc/nv_tegra_release)

            case $r_number in
                R35)
                    # Jetson Orin
                    compose_args="$compose_args -f docker-compose.yml -f docker-compose.jetson.orin.yml"
                    ;;
                R32)
                    # Jetson Nano
                    compose_args="$compose_args -f docker-compose.yml -f docker-compose.jetson.yml"
                    ;;
                *)
                    echo "Unsupported Jetson version: $r_number"
                    exit 1
                    ;;
            esac
        fi

        # Check if an Nvidia GPU is available
        ( nvidia-smi > /dev/null 2>&1 ) && compose_args="$compose_args -f docker-compose.yml -f docker-compose.nvidia.yml"
        ;;
    Darwin)
        compose_args="$compose_args -f docker-compose.yml -f docker-compose.mac.yml"
        ;;
    *)
        echo "Unsupported OS: $os"
        exit 1
esac
export DOCKER_BUILDKIT=1

if [ -d /usr/local/zed ]; then
    if [ -d ../zedxmini ]; then
        compose_args="$compose_args -f docker-compose.jetson.orin.zedxmini.yml"
    else
        echo -e "\033[33mWARNING:\033[0m Zed X Mini not found. https://github.com/zauberzeug/zedxmini"
    fi
fi

cmd=$1
cmd_args=${@:2}
set -x
case $cmd in
    b | build)
        docker-compose $compose_args pull detector
    	docker-compose $compose_args build --no-cache $cmd_args
        ;;
    u | up)
        docker-compose $compose_args up -d $cmd_args
        ;;
    p | pull)
        docker-compose $compose_args pull
        ;;
    U | uppull)
        docker-compose $compose_args pull
        docker-compose $compose_args up -d --build $cmd_args
        ;;
    d | down)
        docker-compose $compose_args down -d $cmd_args
        ;;
    s | start)
        docker-compose $compose_args start $cmd_args
        ;;
    r | restart)
        docker-compose $compose_args restart $cmd_args
        ;;
    h | stop)
        docker-compose $compose_args stop $cmd_args
        ;;
    i | install)
        echo "disabing restart for any containers which may have been configured before"
        docker update --restart=no $(docker ps -a -q) > /dev/null
        echo "configuring running containers to always restart"
        docker update --restart=always $(docker ps -q)
        ;;
    rm)
        docker-compose $compose_args rm $cmd_args
        ;;
    ps)
        docker-compose $compose_args ps $cmd_args
        ;;
    stat | stats)
        docker stats $cmd_args
        ;;
    l | log | logs)
        docker-compose $compose_args logs -f --tail 1000 $cmd_args
        ;;
    e | exec)
        docker-compose $compose_args exec $cmd_args
        ;;
    a | attach)
        docker-compose $compose_args exec $cmd_args /bin/bash
        ;;
    prune)
        docker system prune
        ;;
    stopall)
        docker stop $(docker ps -aq)
        ;;
    killall)
        docker kill $(docker ps -aq)
        ;;
    *)
        echo "Unsupported command \"$cmd\""
        exit 1
esac

