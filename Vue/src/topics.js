// 监控话题清单 —— 集中定义「监控什么 + 怎么判断数据是否正常」。
// 每一条都是前端数据模型的单一来源：rosbridge 订阅、解析、健康判定都从这里驱动。

// 每个话题的解析器。返回一个 { ok, value, detail } 结构：
//   ok     - 解析是否成功（false 会在页面上标红该环节）
//   value  - 归一化后用于展示/绘图的核心数值
//   detail - 用于卡片副标题 / 状态判断的补充字段
const parsers = {
  // ---------- GPS / RTK ----------
  '/gps/fix': (m) => {
    const s = m.status && m.status.status
    const fixOk = m.status && m.status.status >= 0 && !isNaN(m.latitude) && !isNaN(m.longitude)
    return {
      ok: fixOk && m.status && m.status.status >= 0,
      value: m.latitude !== undefined && m.longitude !== undefined
        ? [m.latitude, m.longitude] : null,
      detail: {
        lat: m.latitude,
        lon: m.longitude,
        alt: m.altitude,
        fix: s === 2 ? 'GPS 固定' : s === 1 ? 'RTK 浮动' : s === 0 ? '无定位' : `状态 ${s}`,
      },
    }
  },
  '/gps/heading': (m) => {
    const deg = (m.data * 180) / Math.PI
    return { ok: isFinite(m.data), value: deg, detail: { deg: deg.toFixed(2) } }
  },
  '/gps/status': (m) => {
    let parsed = null
    try { parsed = JSON.parse(m.data) } catch (e) { /* ignore */ }
    return {
      ok: parsed !== null,
      value: parsed ? (parsed.heartbeat_received ? 1 : 0) : 0,
      detail: parsed || { heartbeat_received: false },
    }
  },

  // ---------- IMU / 里程计 ----------
  '/unilidar/imu': (m) => {
    const q = m.orientation || {}
    return {
      ok: isFinite(q.w),
      value: [q.x, q.y, q.z, q.w],
      detail: {
        ang: m.angular_velocity,
        acc: m.linear_acceleration,
      },
    }
  },
  '/aft_mapped_to_init': (m) => {
    const p = m.pose && m.pose.pose && m.pose.pose.position
    return {
      ok: p && isFinite(p.x) && isFinite(p.y),
      value: p ? [p.x, p.y, p.z] : null,
      detail: { x: p ? p.x : null, y: p ? p.y : null, z: p ? p.z : null },
    }
  },

  // ---------- 路径 / 目标 ----------
  '/plan': (m) => {
    const n = (m.poses || []).length
    const last = n ? m.poses[n - 1].pose.position : null
    return {
      ok: n > 0,
      value: n,
      detail: { points: n, end: last ? [last.x, last.y] : null },
    }
  },
  '/goal_pose': (m) => {
    const p = m.pose && m.pose.position
    return {
      ok: p && isFinite(p.x),
      value: p ? [p.x, p.y] : null,
      detail: { x: p ? p.x : null, y: p ? p.y : null },
    }
  },
  '/plan_geodetic': (m) => {
    const n = (m.poses || []).length
    const first = n ? m.poses[0].pose.position : null
    const last = n ? m.poses[n - 1].pose.position : null
    return {
      ok: n > 0,
      value: n,
      detail: { points: n, first: first, last: last },
    }
  },
  '/usv/goal_status': (m) => {
    const ok = typeof m.data === 'string' && m.data.length > 0
    return { ok, value: ok ? m.data.length : 0, detail: { text: m.data } }
  },

  // ---------- 航点链路 ----------
  '/target/fix': (m) => {
    const ok = m.status && m.status.status >= 0 && isFinite(m.latitude)
    return {
      ok,
      value: ok ? [m.latitude, m.longitude] : null,
      detail: { lat: m.latitude, lon: m.longitude },
    }
  },
  '/target/geopath': (m) => {
    const n = (m.poses || []).length
    return { ok: n > 0, value: n, detail: { points: n } }
  },
  '/goal_poses': (m) => {
    const n = (m.poses || []).length
    return { ok: n > 0, value: n, detail: { points: n } }
  },

  // ---------- 推进 / 安全 ----------
  '/wamv/thrusters/left/thrust': (m) => ({ ok: isFinite(m.data), value: m.data }),
  '/wamv/thrusters/right/thrust': (m) => ({ ok: isFinite(m.data), value: m.data }),
  '/usv/safety_stop': (m) => ({ ok: typeof m.data === 'boolean', value: m.data ? 1 : 0 }),
}

// 完整监控清单。每个话题的元数据：
//   topic   - ROS 话题名
//   type    - ROS 消息类型（rosbridge 订阅需要）
//   group   - 分组（用于拓扑图/卡片分区）
//   label   - 中文名
//   parser  - 上面的解析器
//   chart   - 是否要画时序折线图（true 的话 value 必须是数值）
export const TOPICS = [
  { topic: '/gps/fix', type: 'sensor_msgs/msg/NavSatFix', group: 'GPS/航向', label: 'GPS 定位', parser: 'gps/fix', chart: false },
  { topic: '/gps/heading', type: 'std_msgs/msg/Float64', group: 'GPS/航向', label: 'RTK 航向', parser: 'heading', chart: true },
  { topic: '/gps/status', type: 'std_msgs/msg/String', group: 'GPS/航向', label: 'GPS 状态', parser: 'status', chart: false },

  { topic: '/unilidar/imu', type: 'sensor_msgs/msg/Imu', group: 'IMU/里程计', label: 'IMU 姿态', parser: 'imu', chart: false },
  { topic: '/aft_mapped_to_init', type: 'nav_msgs/msg/Odometry', group: 'IMU/里程计', label: 'LIO 里程计', parser: 'odom', chart: false },

  { topic: '/plan', type: 'nav_msgs/msg/Path', group: '路径/目标', label: '规划路径', parser: 'plan', chart: true },
  { topic: '/goal_pose', type: 'geometry_msgs/msg/PoseStamped', group: '路径/目标', label: '目标点', parser: 'goal', chart: false },
  { topic: '/plan_geodetic', type: 'geographic_msgs/msg/GeoPath', group: '路径/目标', label: 'WGS84 路径', parser: 'geopath', chart: false },
  { topic: '/usv/goal_status', type: 'std_msgs/msg/String', group: '路径/目标', label: '目标状态', parser: 'goal_status', chart: false },

  { topic: '/target/fix', type: 'sensor_msgs/msg/NavSatFix', group: '航点链路', label: '目标经纬度', parser: 'target', chart: false },
  { topic: '/target/geopath', type: 'geographic_msgs/msg/GeoPath', group: '航点链路', label: '航点串', parser: 'target_geopath', chart: false },
  { topic: '/goal_poses', type: 'geometry_msgs/msg/PoseArray', group: '航点链路', label: '米制航点', parser: 'goal_poses', chart: false },

  { topic: '/wamv/thrusters/left/thrust', type: 'std_msgs/msg/Float64', group: '推进/安全', label: '左推力', parser: 'thr_left', chart: true },
  { topic: '/wamv/thrusters/right/thrust', type: 'std_msgs/msg/Float64', group: '推进/安全', label: '右推力', parser: 'thr_right', chart: true },
  { topic: '/usv/safety_stop', type: 'std_msgs/msg/Bool', group: '推进/安全', label: '急停', parser: 'safety', chart: false },
]

// 解析器查找表
const parserMap = {
  'gps/fix': parsers['/gps/fix'],
  heading: parsers['/gps/heading'],
  status: parsers['/gps/status'],
  imu: parsers['/unilidar/imu'],
  odom: parsers['/aft_mapped_to_init'],
  plan: parsers['/plan'],
  goal: parsers['/goal_pose'],
  geopath: parsers['/plan_geodetic'],
  goal_status: parsers['/usv/goal_status'],
  target: parsers['/target/fix'],
  target_geopath: parsers['/target/geopath'],
  goal_poses: parsers['/goal_poses'],
  thr_left: parsers['/wamv/thrusters/left/thrust'],
  thr_right: parsers['/wamv/thrusters/right/thrust'],
  safety: parsers['/usv/safety_stop'],
}

export function parseTopic(topic, message) {
  const entry = TOPICS.find((t) => t.topic === topic)
  if (!entry || !parserMap[entry.parser]) return null
  return parserMap[entry.parser](message)
}
