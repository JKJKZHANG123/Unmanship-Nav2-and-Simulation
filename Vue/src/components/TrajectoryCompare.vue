<script setup>
import { ref, watch, onMounted, onBeforeUnmount } from 'vue'
import * as echarts from 'echarts'

const props = defineProps({
  trajectory: { type: Object, required: true },
})

const el = ref(null)
let chart = null
let timer = null

function buildOption(traj) {
  const gpsPts = (traj.gps.points || []).map((p) => [p.x, p.y])
  const lioPts = (traj.lio.points || []).map((p) => [p.x, p.y])
  return {
    backgroundColor: 'transparent',
    tooltip: { trigger: 'axis', valueFormatter: (v) => v.map((n) => n.toFixed(2)).join(', ') },
    legend: { textStyle: { color: '#94a3b8' }, top: 0 },
    grid: { left: 50, right: 20, top: 40, bottom: 30 },
    xAxis: {
      type: 'value',
      name: '东向 (m)',
      nameTextStyle: { color: '#64748b' },
      axisLabel: { color: '#64748b' },
      splitLine: { lineStyle: { color: '#1e293b' } },
    },
    yAxis: {
      type: 'value',
      name: '北向 (m)',
      nameTextStyle: { color: '#64748b' },
      axisLabel: { color: '#64748b' },
      splitLine: { lineStyle: { color: '#1e293b' } },
    },
    series: [
      {
        name: 'GPS 轨迹',
        type: 'line',
        data: gpsPts,
        showSymbol: false,
        lineStyle: { color: '#38bdf8', width: 2 },
      },
      {
        name: 'LIO 轨迹',
        type: 'line',
        data: lioPts,
        showSymbol: false,
        lineStyle: { color: '#34d399', width: 2 },
      },
    ],
  }
}

function render() {
  if (chart && props.trajectory) chart.setOption(buildOption(props.trajectory), true)
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
watch(() => props.trajectory, render, { deep: true })
</script>

<template>
  <div ref="el" class="chart-wrap tall"></div>
</template>
