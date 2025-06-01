#!/bin/bash
source devel/setup.bash
rostopic pub -1 /takeoff_land icaa_uav_se3_controller/TakeoffLand "takeoff_land_cmd: 2"