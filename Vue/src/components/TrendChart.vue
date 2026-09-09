<script setup>
import { ref, watch, onMounted, onBeforeUnmount } from 'vue'
import * as echarts from 'echarts'

const props = defineProps({
  topics: { type: Object, required: true },
})

const el = ref(null)
let chart = null
let timer = null

// 只对标记了 chart:true 且数值型的话题画时序
const CHART_TOPICS = [
  { topic: '/gps/heading', label: 'RTK 航向(°)' },
  { topic: '/plan', label: '规划路径点数' },
  { topic: '/wamv/thrusters/left/thrust', label: '左推力(N)' },
  { topic: '/wamv/thrusters/right/thrust', label: '右推力(N)' },
]

function buildOption(topics) {
  const series = []
  const now = Date.now()
  for (const c of CHART_TOPICS) {
    const e = topics[c.topic]
    const data = e && e.history ? e.history.map((p) => [p.t, p.v]) : []
    series.push({
      name: c.label,
      type: 'line',
      showSymbol: false,
      smooth: true,
      data,
    })
  }
  return {
    backgroundColor: 'transparent',
    tooltip: { trigger: 'axis' },
    legend: { textStyle: { color: '#94a3b8' }, top: 0 },
    grid: { left: 50, right: 20, top: 40, bottom: 30 },
    xAxis: {
      type: 'time',
      axisLabel: { color: '#64748b', formatter: (v) => new Date(v).toLocaleTimeString('zh-CN', { hour12: false }) },
      splitLine: { lineStyle: { color: '#1e293b' } },
    },
    yAxis: {
      type: 'value',
      axisLabel: { color: '#64748b' },
      splitLine: { lineStyle: { color: '#1e293b' } },
    },
    series,
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
  <div ref="el" class="chart-wrap"></div>
</template>
