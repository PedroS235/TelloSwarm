#!/bin/bash
# wrapper for static_transform_publisher, to allow fetching pose from param server
# usage: static_transform.sh TRANSFORM SOURCE TARGET
rosparam get $1 && rosrun tf2_ros static_transform_publisher $(rosparam get $1 | tr '\n' ',' | tr -d - | rev | cut -c 3- | rev) $2 $3


