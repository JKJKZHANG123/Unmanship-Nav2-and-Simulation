// 参数读写 —— 基于 rosapi 的 GetParam / SetParam / GetParamNames 服务。
// 集中定义「哪些参数值得读写」：分组、节点命名空间、参数名、类型、说明。
//
// 每个参数的 default 是 YAML 配置文件里的值（即「出厂值」）。
// 「恢复出厂」= 把运行时参数写回 default（不碰 YAML 文件，等价于重启节点后的值）。

export const PARAM_GROUPS = [
  {
    title: '船体与避障',
    node: '/controller_server',
    params: [
      {
        name: 'footprint',
        type: 'string',
        desc: '船体轮廓（多边形点串）',
        section: 'local_costmap/local_costmap',
        default: '[[0.792302, 0.295345], [0.792302, -0.324655], [-0.417698, -0.324655], [-0.417698, 0.295345]]',
      },
      {
        name: 'inflation_radius',
        type: 'number',
        desc: '障碍膨胀半径',
        section: 'local_costmap/local_costmap',
        default: 3.0,
        unit: 'm',
        min: 0.5,
        max: 20,
        step: 0.1,
      },
    ],
  },
  {
    title: '航行速度',
    node: '/controller_server',
    params: [
      {
        name: 'desired_linear_vel',
        type: 'number',
        desc: '目标巡航速度',
        default: 0.6,
        unit: 'm/s',
        min: 0.1,
        max: 2.0,
        step: 0.05,
      },
      {
        name: 'xy_goal_tolerance',
        type: 'number',
        desc: '到达目标点容差',
        default: 2.0,
        unit: 'm',
        min: 0.2,
        max: 5.0,
        step: 0.1,
      },
      {
        name: 'minimum_turning_radius',
        type: 'number',
        desc: '最小转弯半径',
        default: 7.0,
        unit: 'm',
        min: 3.0,
        max: 15.0,
        step: 0.5,
      },
    ],
  },
  {
    title: '目标与安全',
    node: '/click_to_goal',
    params: [
      {
        name: 'max_goal_distance_m',
        type: 'number',
        desc: '最大可下发目标距离',
        default: 45.0,
        unit: 'm',
        min: 5.0,
        max: 200.0,
        step: 1.0,
      },
      {
        name: 'goal_check_radius_m',
        type: 'number',
        desc: '目标点安全校验半径',
        default: 0.95,
        unit: 'm',
        min: 0.3,
        max: 5.0,
        step: 0.05,
      },
      {
        name: 'reject_unknown',
        type: 'bool',
        desc: '拒绝未知代价单元（更保守）',
        default: true,
      },
    ],
  },
  {
    title: '路径规划',
    node: '/geodetic_goal_planner',
    params: [
      {
        name: 'replan_period_s',
        type: 'number',
        desc: '自动重规划周期',
        default: 1.0,
        unit: 's',
        min: 0.5,
        max: 30.0,
        step: 0.5,
      },
      {
        name: 'server_timeout_s',
        type: 'number',
        desc: '规划服务超时',
        default: 2.0,
        unit: 's',
        min: 0.5,
        max: 10.0,
        step: 0.5,
      },
      {
        name: 'max_retries',
        type: 'number',
        desc: '最大重试次数（0=无限）',
        default: 0,
        unit: '次',
        min: 0,
        max: 100,
        step: 1,
      },
    ],
  },
]

// 拉平：每个参数一个完整描述，方便组件遍历
export const ALL_PARAMS = PARAM_GROUPS.flatMap((g) =>
  g.params.map((p) => ({
    group: g.title,
    node: g.node,
    section: p.section || '',
    ...p,
  }))
)

// 组装完整 ROS 参数路径。
export function fullParamName(p, node) {
  const ns = (node || p.node || '').replace(/\/$/, '')
  if (p.section) {
    return `${ns}/${p.section}/${p.name}`
  }
  return `${ns}/${p.name}`
}

// 转 YAML 值字符串（rosapi SetParam 的 value 字段是 string，会做 YAML 解析）
export function toParamString(type, value) {
  if (type === 'string') return String(value)
  if (type === 'bool') return value === true || value === 'true' ? 'true' : 'false'
  if (type === 'number') return String(value)
  return String(value)
}

// 从 GetParam 返回的字符串解析回 JS 值
export function fromParamString(type, str) {
  if (str === undefined || str === null || str === '') return null
  if (type === 'number') {
    const n = Number(str)
    return isFinite(n) ? n : str
  }
  if (type === 'bool') return str === 'true' || str === 'True'
  return str
}
