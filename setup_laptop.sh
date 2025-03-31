#!/bin/bash
echo "Setup unitree G1 ros2 environment"
SCRIPT_DIR=$(dirname "$(realpath "$BASH_SOURCE")")
source $SCRIPT_DIR/cyclonedds_ws/install/setup.bash
source $SCRIPT_DIR/unitree_ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# LAN
# export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
#                             <NetworkInterface name="enp0s31f6" priority="default" multicast="default" />
#                         </Interfaces></General></Domain></CycloneDDS>'

# WLAN
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="wlp0s20f3" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
