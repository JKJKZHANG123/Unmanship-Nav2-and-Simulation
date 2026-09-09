# 变更记录 — 多航点路径规划 + 取消/重发健壮性

日期：2026-09-08（初稿）／ 2026-09-09（交叉审查修订）

本次改动围绕实船迁移后的两个需求：

1. **健壮性**：执行第一个路径任务时，可随时终止并立即重新下发新目标。
2. **多航点路径规划**：只发送航点，系统算出一条连续经过所有航点（按给定顺序）的最优路径。

实船当前为 **path-only 模式**（只规划、不下发推力）。本次改动不改变这一安全姿态，并
进一步把单独启动 `navigation.launch.py` 时的默认行为也收紧为 path-only（见「已知问题」第 5 条）。

---

## 一、阶段一：修复「取消 → 重发」的竞态 bug

**文件**：`src/usv_navigation/usv_navigation/click_to_goal.py`

### 问题

`_on_goal_result` 原先无条件执行 `self._active_goal_handle = None`，在「取消目标 A → 立即发
目标 B」的时序下会把 B 的句柄误清掉，导致 B 之后再也取消不了。

### 修复

引入单调 **goal generation** 计数 + **handle 身份比较**：

- 新增 `self._goal_generation`，每次 `_queue_goal` / `_on_cancel` 时 `+1`；
- 每个 action 回调携带发送时的 generation；
- `_on_goal_result` 只在 `handle is self._active_goal_handle` 时才清句柄；
- `_on_goal_response` 对已被取代的请求，若已 accepted 则 `cancel_goal_async()` 后静默返回；
- **（交叉审查补充）** `_on_goal_response` 的 send-future 异常路径，先检查 generation 是否过期，
  过期则直接 `return`，不发布过期的 `FAILED` 状态。

---

## 二、阶段二：多航点一次规划

**文件**：`src/usv_navigation/usv_navigation/geodetic_goal_planner.py`（重写规划逻辑）

### 改动

改用 Nav2 原生 **`ComputePathThroughPoses`**：一次性把整串有序航点交给 SmacPlannerHybrid，
返回**一条连续经过所有航点的单条路径**（不再是逐段拼接）。

- `action_name` 从 `/compute_path_to_pose` 改为 `/compute_path_through_poses`；
- `Goal.goals = 整串 route`，`use_start = False`（起点用船当前位置）；
- 保留取消/抢占/状态机骨架（generation 计数、`_abort_route`、`_publish_empty_path`）。

> 关键保证：`/plan` 上只会出现「空」或「完整一条」，绝不出现「半条新 + 半条旧」。

### （交叉审查补充）修复两处竞态 + 激活死参数

1. `_on_goal_response` 异常路径先检查 generation，过期请求不再碰当前状态；
2. `_on_result` 把 `generation` 检查**提前**到 `self._busy = False` 之前——过期 action 的迟到
   result 不再误清新路由的 `_busy`，避免 `_tick` 发出重复 action；
3. 激活 `server_timeout_s`：`_tick` 在 `_busy` 超过 `_server_timeout` 时按
   `abort_on_planning_error` 决定 abort（默认）或 retry，并取消挂起的 action。

---

## 三、新增：多航点地理输入（带严格校验）

**文件**：`src/usv_navigation/usv_navigation/target_waypoints_to_goals.py`（新）

接收一串 WGS84 航点（`/target/geopath`），经 UTM 中间帧变换到 `camera_init`，发布成单个
`PoseArray`（`/goal_poses`）。

### （交叉审查补充）安全校验

- `strict_waypoint_validation: true`（默认）——任一航点无效则**整条路线拒绝**，而不是跳过
  无效点规划剩余子集；
- `reject_zone_mismatch: true`（默认）——拒绝跨 UTM zone / 半球 的航点串，并与船当前 zone
  比对；
- `max_waypoint_distance_m`——单个航点到船当前位置的距离上限；
- `max_segment_distance_m`——相邻航点距离上限；
- 先统一校验并转 UTM，再计算中间航点 yaw（原实现直接对下一个点转 UTM，下一个点无效时会
  抛异常）。

---

## 四、配套改动

| 文件 | 改动 |
|------|------|
| `src/usv_navigation/setup.py` | 注册 `target_waypoints_to_goals` 入口点 |
| `src/usv_navigation/launch/geodetic_io.launch.py` | 启动 `target_waypoints_to_goals` 节点 |
| `src/usv_navigation/config/geodetic_io_real.yaml` | ① `action_name` 改为 `/compute_path_through_poses`；② 新增 `target_waypoints_to_goals` 参数段（含严格校验参数）|
| `experiments/send_waypoints.py`（新） | CLI 发航点脚本，米制 `x y x y…` 或 `--geodetic lat lon…` |
| `src/usv_navigation/usv_navigation/target_waypoint_serial.py` | 新增多航点串口解析：`WAYPOINTS,lat,lon,…` 与 `{"waypoints":[...]}`，发布 `/target/geopath` |
| `src/usv_navigation/usv_navigation/geodetic_path_publisher.py` | 空 `/plan` 现在发布空 GeoPath + 空 JSON，传播「清空」信号到下游 |
| `src/usv_navigation/usv_navigation/mavlink_rtk_bridge.py` | 收到空 `/plan_geodetic` 时清掉待上传的 pending mission（不中断在传的，也不主动清飞控 mission）|
| `src/usv_navigation/launch/navigation.launch.py` | `enable_goal_gateway` 默认 `true` → `false`（实船安全）|
| `nav.sh` / `usv.sh` / `test_nav2.sh` | 仿真链显式传 `enable_goal_gateway:=true`，保持仿真行为不变 |
| `src/usv_localization/config/navsat_real.yaml` | `use_odometry_yaw: false`（实船统一用 RTK yaw 锚定朝向）|
| `src/usv_localization/launch/navsat_only.launch.py` | `use_rtk_heading` / `navsat_imu_topic` / `use_odometry_yaw` 默认值与实船对齐 |

---

## 五、验证结果

```bash
colcon build --packages-select usv_navigation --symlink-install   # ✅
python3 -m pytest src/usv_navigation/test/                        # ✅ 19 passed
```

- 当前 pytest 日志显示 **19 个测试全部通过**（含新增 3 个多航点串口解析测试）。
- **未确认** ament_flake8 / ament_pep257 / ament_copyright 是否实际执行；本日志只证明 pytest 通过。
- 本次核心改动**未新增**针对以下链路的专项测试：
  - `click_to_goal` 取消/重发竞态；
  - `geodetic_goal_planner` 多航点规划；
  - `target_waypoints_to_goals` 坐标转换；
  - 空 `/plan` 到下游输出的传播。

---

## 六、用法

```bash
# 米制航点（直接发 /goal_poses）
python3 experiments/send_waypoints.py 10 0 20 5 30 0

# 经纬度航点（发 /target/geopath → target_waypoints_to_goals 转米制）
python3 experiments/send_waypoints.py --geodetic 31.2304 121.4737 31.2310 121.4740

# 串口多航点（target_waypoint_serial 解析）
#   WAYPOINTS,31.2304,121.4737,31.2310,121.4740
#   {"waypoints":[{"lat":31.2304,"lon":121.4737},{"lat":31.2310,"lon":121.4740}]}

# 随时终止 + 清空路径
ros2 topic pub --once /cancel_navigation std_msgs/msg/Empty '{}'
```

### 取消语义（重要）

`/cancel_navigation` 会：
- 清空 `geodetic_goal_planner` 内部 route；
- 发布空 `/plan`；
- 空 `/plan` 经 `geodetic_path_publisher` 传播为空 `/plan_geodetic` + 空 JSON；
- `mavlink_rtk_bridge` 收到空 geopath 后清掉 pending mission。

**不会**主动向飞控发送 `MISSION_CLEAR_ALL` 清空飞控侧 mission（需显式命令，且当前
`mission_upload_enabled` 默认 false）。

---

## 七、航点数量说明

- `experiments/send_waypoints.py` 当前要求至少两个航点；
- `ComputePathThroughPoses` action 接口要求的是**非空** goals 列表；
- 单点 `/goal_pose` 进入 `geodetic_goal_planner` 后也会被包装成**一个元素**的 route，统一走
  `ComputePathThroughPoses` 规划（不是「单点仍走 `/goal_pose` 单点路径」）。

---

## 八、UTM 转换结论（已核实）

- `robot_localization` 的 `navsat_transform_node` 在当前配置下
  （`use_local_cartesian: false` + `broadcast_utm_transform: true`）广播的 `utm` frame 是
  **绝对 UTM 坐标**，不是相对 datum 的局部坐标；
- 项目 `geodesy.lat_lon_to_utm` 生成的也是绝对 UTM easting/northing，两者语义兼容；
- 转换思路 `经纬度 → 绝对 UTM → utm/camera_init TF → camera_init` 正确；
- **前提**：所有航点必须与 navsat_transform 当前 UTM zone 一致。`target_waypoints_to_goals`
  现已加入 zone/hemisphere 校验（见第三节）。

---

## 九、已知问题（交叉审查 + 本次修订后仍存在）

1. **MAVLink mission 上传超时后去重**：`mavlink_rtk_bridge` 超时丢弃后不清
   `_mission_received_signature`，若飞控短暂无响应且后续路径签名恰好相同，该路径会被去重
   忽略、直到内容变化才恢复。**（未改动，用户已确认暂不加固）**
2. **取消不主动清飞控 mission**：`/cancel_navigation` 不会向飞控发 `MISSION_CLEAR_ALL`，飞控
   侧旧 mission 需操作者或后续协议显式清除。取消后若重发**内容完全相同**的新任务，也会被
   `_mission_received_signature` 去重忽略（与第 1 条同源）。
3. **串口多航点已支持但未实测**：`target_waypoint_serial` 的多航点解析已实现并有单元测试，
   但未在真实串口/飞控链路上实测。
4. **`abort_on_planning_error` 已生效，但 `server_timeout_s` 只覆盖 result 超时**：action
   server 的 ack 等待超时未单独实现，仍依赖 `retry_period_s` 轮询。
5. **`navigation.launch.py` 单独启动**：默认 `enable_goal_gateway=false`（安全），仿真链已显式
   传 `:=true`。单独手动启动时若要执行目标需显式 `enable_goal_gateway:=true`。
6. **`real_boat_system.launch.py` 与 `navsat_only.launch.py` 的 yaw 参数已统一**为「实船用 RTK
   yaw」，但 `navsat.yaml`（仿真 EKF 链）仍 `use_odometry_yaw: true`，属**独立的正确路径**，
   勿混淆。
7. **单点与多点的末点 yaw 语义不一致**：`target_to_goal` 单点末点 yaw = 船→目标方位角；
   `target_waypoints_to_goals` 多点末点 yaw = 0。path-only 模式下无影响，若将来 Nav2 执行
   目标则存在朝向一致性差异（设计选择，未改）。
8. **`/goal_pose` 双消费者**：实船 path-only 下 `click_to_goal` 不启动（`enable_goal_gateway=
   false`），`/goal_pose` 仅由 `geodetic_goal_planner` 消费；仿真（`enable_goal_gateway=true`）
   下 `/goal_pose` 会**同时**被 `click_to_goal`（执行）和 `geodetic_goal_planner`（规划）消费，
   属既有的双路径设计。

---

## 十、本轮逐文件审查新增的修复（2026-09-09）

1. **`geodetic_path_publisher` 取消后无限重发空 geopath**：`_retry_latest_plan`（1 Hz）原先
   无条件重发 `_last_plan`，取消后每 1 秒重发空 geopath 并刷屏。修复：重试循环只重发非空
   path。
2. **`geodetic_goal_planner` 重规划被误判为「取消」**：`_tick` 原先在每次周期重规划前
   `_publish_empty_path()`，与「空 path = 取消」的下游语义冲突，导致重规划每 1 秒清一次
   mission。修复：重规划前不再发空 path（规划失败时 `_abort_route` 自会清空）。
3. **`target_waypoints_to_goals` 死代码**：删除 pass 1 里四个 `= None` 的冗余初始化。

---

## 十一、本次交叉审查的主要修正点

- 修正「至少 2 个航点」的不严谨说法（见第七节）；
- 修正「验证结论写过头」（见第五节）；
- 补充 `/cancel_navigation` 下游传播语义（见第六节）；
- 补充串口多航点输入缺口（第四节已实现）；
- 补充 `target_waypoints_to_goals` 安全校验不足（第三节已修复）；
- 补充 `navigation.launch.py` 默认值隐患（第四节已修复）；
- 修复 `geodetic_goal_planner` 过期 result 误清 `_busy`、`click_to_goal` send-future 异常路径
  两处真实竞态（第一、二节）；
- 激活 `server_timeout_s` / `abort_on_planning_error`（第二节）。
