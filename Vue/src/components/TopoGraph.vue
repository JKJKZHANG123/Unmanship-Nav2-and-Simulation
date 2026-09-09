<script setup>
import { ref, watch, onMounted, onBeforeUnmount } from 'vue'
import * as echarts from 'echarts'

const props = defineProps({
  topics: { type: Object, required: true },
})

const el = ref(null)
let chart = null
let timer = null

// 数据流拓扑 —— 节点是 ROS 话题，边是「生产者 -> 消费者」关系。
// 实船 + 仿真的关键链路。每个话题的实时健康状态决定节点颜色。
const NODES = [
  // 传感器层
  { name: '/unilidar/imu', x: 80, y: 60, group: '传感器' },
  { name: '/unilidar/cloud', x: 80, y: 160, group: '传感器' },
  { name: '/gps/fix', x: 80, y: 260, group: '传感器' },
  { name: '/gps/heading', x: 80, y: 360, group: '传感器' },

  // 定位/建图层
  { name: '/aft_mapped_to_init', x: 300, y: 110, group: '定位' },
  { name: '/gps/heading_imu', x: 300, y: 260, group: '定位' },

  // 航点/目标层
  { name: '/target/fix', x: 520, y: 60, group: '航点' },
  { name: '/target/geopath', x: 520, y: 160, group: '航点' },
  { name: '/goal_poses', x: 520, y: 260, group: '航点' },
  { name: '/goal_pose', x: 520, y: 360, group: '航点' },

  // 路径层
  { name: '/plan', x: 740, y: 160, group: '路径' },
  { name: '/plan_geodetic', x: 740, y: 300, group: '路径' },
  { name: '/usv/goal_status', x: 740, y: 420, group: '路径' },

  // 执行/安全层
  { name: '/wamv/thrusters/left/thrust', x: 960, y: 200, group: '执行' },
  { name: '/wamv/thrusters/right/thrust', x: 960, y: 300, group: '执行' },
  { name: '/usv/safety_stop', x: 960, y: 400, group: '执行' },
]

const EDGES = [
  ['/unilidar/imu', '/aft_mapped_to_init'],
  ['/unilidar/cloud', '/aft_mapped_to_init'],
  ['/gps/heading', '/gps/heading_imu'],
  ['/unilidar/imu', '/gps/heading_imu'],
  ['/gps/fix', '/target/fix'],
  ['/target/fix', '/target/geopath'],
  ['/target/geopath', '/goal_poses'],
  ['/target/fix', '/goal_pose'],
  ['/goal_poses', '/plan'],
  ['/goal_pose', '/plan'],
  ['/plan', '/plan_geodetic'],
  ['/plan_geodetic', '/usv/goal_status'],
  ['/aft_mapped_to_init', '/plan'],
  ['/plan', '/wamv/thrusters/left/thrust'],
  ['/plan', '/wamv/thrusters/right/thrust'],
  ['/plan', '/usv/safety_stop'],
]

const GROUP_COLORS = {
  '传感器': '#38bdf8',
  '定位': '#a78bfa',
  '航点': '#f472b6',
  '路径': '#34d399',
  '执行': '#fbbf24',
}

function topicState(topics, name) {
  const e = topics[name]
  if (!e || e.lastStamp === 0) return 'stale'
  if (!e.healthy) return 'err'
  return 'ok'
}

function buildOption(topics) {
  const nodeMap = {}
  const nodes = NODES.map((n) => {
    const st = topicState(topics, n.name)
    nodeMap[n.name] = st
    const color = st === 'ok' ? GROUP_COLORS[n.group] : st === 'err' ? '#f87171' : '#4b5563'
    return {
      name: n.name,
      x: n.x,
      y: n.y,
      symbolSize: st === 'ok' ? 18 : 14,
      itemStyle: { color },
      label: { show: true, position: 'right', fontSize: 11, color: '#e2e8f0' },
    }
  })
  const links = EDGES.map(([s, t]) => {
    const ok = nodeMap[s] === 'ok' && nodeMap[t] === 'ok'
    return {
      source: s,
      target: t,
      lineStyle: { color: ok ? '#334155' : '#1f2937', width: 1.5, curveness: 0.1 },
    }
  })
  return {
    backgroundColor: 'transparent',
    tooltip: { show: true, formatter: (p) => p.dataType === 'node' ? p.name : `${p.data.source} → ${p.data.target}` },
    series: [
      {
        type: 'graph',
        layout: 'none',
        data: nodes,
        links,
        roam: true,
        lineStyle: { color: 'source' },
        emphasis: { focus: 'adjacency', lineStyle: { width: 3 } },
      },
    ],
  }
}

function render() {
  if (chart && props.topics) {
    chart.setOption(buildOption(props.topics), true)
  }
}

onMounted(() => {
  chart = echarts.init(el.value)
  render()
  timer = setInterval(render, 1000)
  window.addEventListener('resize', resize)
})

function resize() { chart && chart.resize() }
onBeforeUnmount(() => {
  clearInterval(timer)
  window.removeEventListener('resize', resize)
  chart && chart.dispose()
})

watch(() => props.topics, render, { deep: true })
</script>

<template>
  <div ref="el" class="topo-wrap"></div>
</template>
