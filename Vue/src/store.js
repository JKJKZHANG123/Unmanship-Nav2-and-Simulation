// 响应式数据存储 + 健康判定。
// 每个话题维护三样东西：
//   latest   - 最近一条解析结果
//   healthy  - 是否「正常」（解析成功 + 未超时）
//   history  - 数值型话题的时序缓冲（供折线图）
import { reactive } from 'vue'
import { TOPICS, parseTopic } from './topics.js'

const STALE_MS = 3000        // 超过 3 秒没收到数据判定为「超时/断流」
const HISTORY_LEN = 120      // 折线图保留最近 120 个点

export function createStore() {
  const topics = reactive({})
  const stats = reactive({
    connected: false,
    total: TOPICS.length,
    healthyCount: 0,
    staleCount: 0,
    errorCount: 0,
  })
  const trajectory = reactive({
    gps: { origin: null, points: [] },
    lio: { points: [] },
  })

  for (const t of TOPICS) {
    topics[t.topic] = {
      label: t.label,
      group: t.group,
      type: t.type,
      chart: t.chart,
      latest: null,        // { ok, value, detail, stamp }
      healthy: false,
      lastStamp: 0,
      history: [],         // [{ t, v }]
    }
  }

  function onMessage(topic, msg) {
    const entry = topics[topic]
    if (!entry) return
    const parsed = parseTopic(topic, msg)
    const now = Date.now()
    entry.latest = parsed
    entry.lastStamp = now
    entry.healthy = !!(parsed && parsed.ok)

    if (entry.chart && parsed && typeof parsed.value === 'number' && isFinite(parsed.value)) {
      entry.history.push({ t: now, v: parsed.value })
      if (entry.history.length > HISTORY_LEN) entry.history.shift()
    }

    // GPS 轨迹（绝对经纬度 -> 局部 XY，用第一个点为原点）
    if (topic === '/gps/fix' && parsed && parsed.value) {
      const [lat, lon] = parsed.value
      if (!trajectory.gps.origin) trajectory.gps.origin = { lat, lon }
      // 简单平面投影（短距离足够）：x=经度差*111320*cos(lat), y=纬度差*111320
      const o = trajectory.gps.origin
      const x = (lon - o.lon) * 111320 * Math.cos((o.lat * Math.PI) / 180)
      const y = (lat - o.lat) * 111320
      trajectory.gps.points.push({ x, y })
      if (trajectory.gps.points.length > 2000) trajectory.gps.points.shift()
    }

    // LIO 轨迹（camera_init 局部 XY）
    if (topic === '/aft_mapped_to_init' && parsed && parsed.value) {
      const [x, y, z] = parsed.value
      trajectory.lio.points.push({ x, y })
      if (trajectory.lio.points.length > 2000) trajectory.lio.points.shift()
    }
    recomputeStats()
  }

  function recomputeStats() {
    const now = Date.now()
    let healthy = 0, stale = 0, error = 0
    for (const t of TOPICS) {
      const e = topics[t.topic]
      const timedOut = now - e.lastStamp > STALE_MS
      if (timedOut || e.lastStamp === 0) {
        e.healthy = false
        stale++
      } else if (!e.healthy) {
        error++
      } else {
        healthy++
      }
    }
    stats.healthyCount = healthy
    stats.staleCount = stale
    stats.errorCount = error
  }

  // 定时器驱动健康判定刷新（即使没有新消息也要更新超时状态）
  setInterval(recomputeStats, 1000)

  return { topics, stats, trajectory, onMessage }
}
