#!/usr/bin/env bash
# USV 数据监控台一键启动：rosbridge_server + Vue 前端开发服务器。
#
# 用法：
#   ./start_monitor.sh            # 前台启动（Ctrl-C 退出）
#   ./start_monitor.sh dev        # 前端用 vite dev（开发，热更新）
#   ./start_monitor.sh build      # 前端用 vite preview（生产构建后的产物）
#
# 依赖：ros-jazzy-rosbridge-server 已安装；Vue 依赖已 npm install。
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:-dev}"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

cleanup() {
  echo ""
  echo "[monitor] 正在停止…"
  kill 0 2>/dev/null || true
}
trap cleanup INT TERM

echo "[monitor] ① 启动 rosbridge_server (WebSocket :9090) …"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# 等 rosbridge 起来（最多 15 秒）
echo "[monitor] 等待 rosbridge 端口就绪…"
for i in $(seq 1 30); do
  if (echo > /dev/tcp/127.0.0.1/9090) 2>/dev/null; then
    echo "[monitor] rosbridge 已就绪 (127.0.0.1:9090)"
    break
  fi
  sleep 0.5
  if [ "$i" = "30" ]; then
    echo "[monitor] ⚠ rosbridge 端口未就绪，请检查 rosbridge_server 是否正常"
  fi
done

echo "[monitor] ② 启动 Vue 前端 …"
cd "$SCRIPT_DIR"
if [ "$MODE" = "build" ]; then
  npm run preview -- --host 0.0.0.0 &
else
  npm run dev &
fi

echo ""
echo "======================================================"
echo " 监控台已启动："
echo "   浏览器访问  http://<jetson-ip>:5173"
echo "   本机访问    http://127.0.0.1:5173"
echo "   停止        Ctrl-C"
echo "======================================================"
echo ""

wait
