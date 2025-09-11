# ENSURING RUNNING WITH SUDO (or permission issues will occur when sched_attr)
if [ -z "$SUDO_USER" ]; then
  echo "script failed: run with sudo"
  return
fi

export AMENT_PREFIX_PATH=""
export CMAKE_PREFIX_PATH=""

. /home/alexy/ros2_humble/install/setup.sh
. /home/alexy/rclcpp/install/setup.sh
# . /home/sizheliu/rclcpp/install/setup.sh

RT_EXEC="$1"
CLEAN="$2"

[ "$CLEAN" = "1" ] && rm -rf build install log
[ "$RT_EXEC" = "1" ] && RT_EXEC=ON || RT_EXEC=OFF

colcon build --packages-up-to node_latency --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit

. ./install/setup.sh
./build/node_latency/main