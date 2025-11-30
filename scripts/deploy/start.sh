#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

# declare mode, use gpu by default
mode="gpu"

# declare sim, use sim by default
sim="True"

while getopts 'ch' opt; do
    case "$opt" in
        c)
            mode="cpu"
            ;;
        ?|h)
            echo "Usage: $(basename $0) [-c]"
            exit 1
            ;;
    esac
done

shift "$(($OPTIND -1))"

HOST_WS=$(dirname "$0")/../../workspace/

if [ "$mode" == "gpu" ]; then
    run_docker --runtime=nvidia \
    -v $HOST_WS:/root/workspace \
    limo_bot:sim "/root/app.sh"
else
    run_docker \
    -v $HOST_WS:/root/workspace \
    limo_bot:sim "/root/app.sh"
fi

sleep 1 #wait a little bit

#plot data after exiting the container if the --plot flag is given
if [ "$1" == "--plot" ]; then
    echo "Generating plots..."
    python3 ./workspace/plot_data.py
fi