#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
export _colcon_cd_root=/opt/ros/humble/
export QT_QPA_PLATFORM=xcb

echo "Provided arguments: $@"
exec $@
