#!/bin/bash
set -e
# ============================================
# 华台协同调姿 — 编译 & 运行脚本
# ============================================
WORKSPACE_ROOT="$(cd "$(dirname "$0")" && pwd)"
ROS_DOMAIN_ID=36
ROS2_SETUP="/opt/ros/humble/setup.bash"
CALIB_SETUP="/home/nkk/tiaozi_calibration/ros2_tiaozi_0605_v2/install/setup.bash"

CLEAN=false
LAUNCH_UI=false
for arg in "$@"; do
    case "$arg" in clean) CLEAN=true ;; ui) LAUNCH_UI=true ;; esac
done

echo "============================================"
echo "  华台协同调姿控制节点"
echo "============================================"

source "${ROS2_SETUP}"
echo "[OK] ROS 2 Humble"

# 必须加载工作项目的 base_interfaces_demo (类型与滑台控制器匹配)
if [ -f "${CALIB_SETUP}" ]; then
    source "${CALIB_SETUP}"
    echo "[OK] 滑台消息类型"
fi

unset RMW_IMPLEMENTATION  # 用默认 Fast-DDS, 与工作项目一致

cd "${WORKSPACE_ROOT}"
if $CLEAN; then
    rm -rf build install log
    echo "[OK] 清理完成"
fi

colcon build --symlink-install
echo "[OK] 编译完成"

source install/setup.bash
export ROS_DOMAIN_ID=36
unset RMW_IMPLEMENTATION

if $LAUNCH_UI; then
    python3 scripts/ui_server.py &
    echo "[OK] Web UI: http://$(hostname -I | awk '{print $1}'):5000"
    trap "kill %1 2>/dev/null" EXIT
fi

echo ""
echo ">>> 启动..."
exec ros2 run demo_cpp_pkg tiaozi_gui
