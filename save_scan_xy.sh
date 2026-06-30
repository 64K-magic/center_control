#!/bin/bash
# 保存一帧 /scan 的 XY 投影图
# 用法：
#   ./save_scan_xy.sh                    # 保存到 ./scan_xy.png
#   ./save_scan_xy.sh /tmp/my_scan.png   # 指定输出路径

set -e

cd "$(dirname "$0")"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

source /opt/ros/humble/setup.bash
source install/setup.bash

OUT_PATH="${1:-/home/jfby/jfby_ws/scan_xy.png}"

echo "Saving /scan XY projection..."
echo "  output: $OUT_PATH"

python3 "$(dirname "$0")/save_scan_xy.py" "$OUT_PATH"

echo "Done."
