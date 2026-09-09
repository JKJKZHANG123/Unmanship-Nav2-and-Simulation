# 双天线 RTK（飞控 MAVLink）接入说明

当前实船启动链路默认按以下拓扑工作：

```text
双天线 RTK → 飞控 → MAVLink UART → Jetson GPIO UART → mavlink_rtk_bridge
                                              ├─ /gps/fix
                                              ├─ /gps/heading
                                              └─ /gps/status
```

## 接线与串口

- 飞控 TX → Jetson GPIO UART RX
- 飞控 RX → Jetson GPIO UART TX（仅当需要请求 MAVLink 数据流时必须接）
- 飞控 GND → Jetson GND
- 确认两端都是 **3.3 V TTL**；不要接 5 V TTL 或 RS-232 电平。
- 默认设备为 `/dev/ttyTHS1`、波特率 `115200`，实际设备名必须以本机检查结果为准：

```bash
ls -l /dev/serial/by-path/
ls -l /dev/ttyTHS* /dev/ttyUSB*
```

GPIO UART 还必须启用，并关闭占用它的 console/getty；同一串口不能被两个节点同时读取。

## MAVLink 数据

桥接节点监听 `HEARTBEAT`、`GPS_RAW_INT`、`GPS2_RAW`、`GLOBAL_POSITION_INT` 和 `ATTITUDE`。双天线航向优先使用 `GPS_RAW_INT.yaw`/`GPS2_RAW.yaw`，再回退到全局位置航迹角和飞控姿态角。航向发布为 ROS ENU yaw（弧度），而不是 MAVLink 的北向顺时针角度。

飞控端应输出有效的 `GPS_RAW_INT.yaw` 或 `GPS2_RAW.yaw`。如果船头实测方向与输出相反，可启动时设置 `heading_offset_deg:=180.0`。

## 先单独测试 RTK

```bash
source /opt/ros/jazzy/setup.bash
source ~/lidar_project/install/setup.bash
ros2 launch usv_navigation mavlink_rtk.launch.py \
  port:=/dev/ttyTHS1 baud:=115200 request_stream:=true
```

另开终端检查：

```bash
ros2 topic echo /gps/fix
ros2 topic echo /gps/heading
ros2 topic echo /gps/heading_source
ros2 topic echo /gps/status
```

`/gps/status` 是 JSON 字符串，至少应看到 `heartbeat_received:true`、有效 `fix_type` 和持续更新的 `heading_source`。其中 `parser.valid_frames` 应持续增加，`parser.crc_errors` 应保持为 0（偶发误码除外）。硬件接线完成并不代表立即可用：飞控 MAVLink 端口配置、波特率、UART 电平、GPS 双天线基线方向和数据流都必须逐项验证。

## 启动完整实船链路

```bash
ros2 launch my_mapping_launcher real_boat_system.launch.py \
  gps_protocol:=mavlink gps_port:=/dev/ttyTHS1 gps_baud:=115200 \
  lidar_port:=/dev/ttyUSB0 rviz:=true
```

该模式会启动 `heading_to_imu`，将 RTK 的绝对航向与 L1 IMU 的横滚/俯仰合成为 `/gps/heading_imu`，并让 `navsat_transform_node` 使用它建立 UTM 到 `camera_init` 的方向锚定。整个工程仍不发送推力；Nav2 只计算并输出路径/速度指令供后续执行器使用。


## Jetson 串口现场确认

当前系统发现 `/dev/ttyTHS1` 和 `/dev/ttyTHS2` 两个硬件 UART，但没有发现
`/dev/serial/by-path/`、USB GPS 或 ACM 设备；这不代表某个 UART 已经接通飞控。
启动前确认 GPIO UART 映射和权限：

```bash
ls -l /dev/ttyTHS* /dev/serial/by-path/
stat -c '%A %U %G %n' /dev/ttyTHS1 /dev/ttyTHS2
id -nG | tr ' ' '\n' | grep '^dialout$'
```

如果账号不在 `dialout` 组，执行 `sudo usermod -aG dialout $USER` 后重新登录。
不要让两个节点同时读取同一串口；先只运行 MAVLink 桥接，再启动完整链路。

若 `heartbeat_received` 一直为 `false`，按顺序检查：实际设备名、TX/RX 交叉、共地、
3.3 V TTL 电平、波特率、飞控端 MAVLink 输出，以及飞控是否把 `GPS_RAW_INT`/`GPS2_RAW`
发送到该遥测口。若有心跳但没有 `GPS_RAW_INT` 航向，需在飞控端启用双天线
moving-baseline yaw 输出；`ATTITUDE` 或 `GLOBAL_POSITION_INT.hdg` 仅是回退，不能替代静止时的双天线绝对航向。

RTK 航向有效时，`heading_to_imu` 会把 L1 IMU 的横滚/俯仰与 RTK yaw 合成
`/gps/heading_imu`。若 RTK 航向超过 2.5 秒未更新，该话题停止发布，避免
`navsat_transform_node` 使用过期航向继续建立错误的 UTM 方向。

## 路径发送给飞控的接口与验证

路径不是通过 `/cmd_vel` 发送，也不会触发解锁、模式切换、推力或舵角输出。当前采用两种输出：

1. **ROS 路径话题**：`/plan` 是 `nav_msgs/Path` 的局部米制路径；`/plan_geodetic` 是包含每个点 WGS84 纬度、经度和高度的 `geographic_msgs/GeoPath`；`/plan_geodetic_json` 是同一条路径的 JSON 字符串。
2. **MAVLink Mission**：将 `mission_upload_enabled:=true` 后，桥接节点按 `MISSION_COUNT → MISSION_REQUEST_INT/REQUEST → MISSION_ITEM_INT/ITEM → MISSION_ACK` 上传最新路径。每个航点包含经纬度、相对高度、航向和到达半径；新规划结果采用 latest-wins，不会把旧路径覆盖新路径。

先只确认 ROS 输出：

```bash
ros2 topic echo /plan_geodetic --once
ros2 topic echo /plan_geodetic_json --once
ros2 topic echo /gps/status --once
```

确认 `gps/status` 中 `heartbeat_received:true`、`mission_state` 和 `mission_points` 正常后，再开启上传：

```bash
ros2 launch my_mapping_launcher real_boat_system.launch.py \
  gps_protocol:=mavlink gps_port:=/dev/ttyTHS1 gps_baud:=115200 \
  target_port:=/dev/ttyUSB2 mission_upload_enabled:=true rviz:=true
```

飞控若要求显式的 `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`，将 `mission_frame:=6`；若使用常规相对高度任务，保持 `mission_frame:=3`。该参数必须以实际飞控（ArduPilot/PX4）和地面站抓包结果为准，首次实船测试应在未解锁、无推进动力条件下观察 `MISSION_ACK` 和任务点内容。
