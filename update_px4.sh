#!/bin/bash
PX4_PATH=~/PX4-Autopilot/Tools/simulation/gz

cp -r worlds/* $PX4_PATH/worlds/

cp -r models/* $PX4_PATH/models/

echo "Files copied, PX4 updated!"