# USV 数据监控台

基于 Vue 3 + ECharts + rosbridge 的实船数据监控网页。浏览器直连 rosbridge WebSocket，
实时展示 ROS 话题数据、判断数据流是否正常，并支持参数读写。

## 功能

1. **数据流向拓扑图** — 15 个话题按「传感器 → 定位 → 航点 → 路径 → 执行」分层连线，
   节点颜色实时表达健康状态（绿=正常，红=解析异常，灰=无数据/超时）。
2. **GPS vs LIO 轨迹对比** — GPS 绝对定位与 LIO 里程计各自连成轨迹线，直观对比漂移。
3. **实时趋势图** — RTK 航向、路径点数、左右推力等数值型话题的时序曲线。
4. **参数读写** — 读取当前参数（footprint、inflation_radius、速度、容差等），修改后写回。
5. **话题数据卡片** — 每个话题一张卡，展示解析状态、核心数值、关键字段。

## 快速开始

```bash
cd /home/jetson/lidar_project/Vue

# 首次：安装依赖
npm install

# 一键启动（rosbridge + 前端）
./start_monitor.sh           # 开发模式（热更新）
./start_monitor.sh build     # 生产构建产物模式

# 浏览器访问
http://<jetson-ip>:5173
```

启动前先跑实船链（`./real_boat.sh` 或 `ros2 launch my_mapping_launcher real_boat_system.launch.py`）。

## 架构

```
ROS 2 (Point-LIO + Nav2 + RTK)
   │  topic: /gps/fix, /unilidar/imu, /aft_mapped_to_init, /plan, ...
   ▼
rosbridge_server (WebSocket :9090)  —— rosbridge_websocket_launch.xml
   │  JSON: subscribe / call_service(rosapi)
   ▼
Vue 前端 (Node 18 + Vite)
   │  src/topics.js     话题清单 + 解析器（单一数据源）
   │  src/rosbridge.js  WebSocket 客户端（手写 rosbridge v2 协议）
   │  src/store.js      响应式状态 + 健康判定 + 轨迹缓冲
   │  src/params.js     参数读写清单
   └  components/       拓扑图 / 轨迹 / 趋势 / 卡片 / 参数面板
```

## 数据接入方式

- 开发模式：vite dev server 把 `/rosbridge` 代理到 `ws://127.0.0.1:9090`。
- 生产模式：`npm run preview` 同样代理；若部署到其它机器，改 `App.vue` 的 `WS_URL`
  指向 `ws://<rosbridge-host>:9090`。

## 参数读写说明

- 底层用 rosapi 的 `get_param` / `set_param` 服务。
- 参数完整名 = 节点命名空间 + 参数名，例如：
  - `/controller_server/desired_linear_vel`
  - `/controller_server/local_costmap/local_costmap/footprint`
- `footprint` 是字符串，`SetParam` 的 `value` 字段直接传字符串，rosapi 内部做 YAML 解析。

## 监控话题清单

| 分组 | 话题 | 类型 |
|------|------|------|
| GPS/航向 | `/gps/fix` `/gps/heading` `/gps/status` | NavSatFix / Float64 / String |
| IMU/里程计 | `/unilidar/imu` `/aft_mapped_to_init` | Imu / Odometry |
| 路径/目标 | `/plan` `/goal_pose` `/plan_geodetic` `/usv/goal_status` | Path / PoseStamped / GeoPath / String |
| 航点链路 | `/target/fix` `/target/geopath` `/goal_poses` | NavSatFix / GeoPath / PoseArray |
| 推进/安全 | `/wamv/thrusters/{left,right}/thrust` `/usv/safety_stop` | Float64 / Bool |

## 排查

- 拓扑节点一直灰：该话题没数据，用 `ros2 topic hz <topic>` 确认。
- 拓扑节点红：解析失败，检查 `topics.js` 里该话题的解析器是否匹配真实消息字段。
- 参数读不出来：确认节点已启动、参数完整名正确（`ros2 param list` 核对）。
- rosbridge 连不上：确认 `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` 已起，
  端口 9090 可达。
