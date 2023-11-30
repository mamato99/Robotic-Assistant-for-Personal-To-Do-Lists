#!/bin/bash

WS_DIR=$( cd -- "$(dirname "${0}")" >/dev/null 2>&1 ; pwd -P )

cd "$WS_DIR/../../.."

source /opt/ros/noetic/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch