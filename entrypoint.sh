set -e

source /opt/ros/humble/setup.bash

WS=/root/ros2/ws
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
    echo "Workspace sourced!"
fi

exec "$@"