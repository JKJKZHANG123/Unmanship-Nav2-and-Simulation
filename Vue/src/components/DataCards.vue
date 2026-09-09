<script setup>
import { computed } from 'vue'

const props = defineProps({
  topics: { type: Object, required: true },
})

// 按分组归类，保持拓扑图里的分组顺序
const GROUP_ORDER = ['GPS/航向', 'IMU/里程计', '路径/目标', '航点链路', '推进/安全']

const groups = computed(() => {
  const map = {}
  for (const g of GROUP_ORDER) map[g] = []
  for (const [topic, e] of Object.entries(props.topics)) {
    if (map[e.group]) map[e.group].push({ topic, ...e })
  }
  return map
})

function stateClass(e) {
  if (e.lastStamp === 0) return 'stale'
  if (!e.healthy) return 'err'
  return 'ok'
}

function stateText(e) {
  if (e.lastStamp === 0) return '无数据'
  if (!e.healthy) return '解析异常'
  return '正常'
}

function fmtValue(e) {
  const v = e.latest && e.latest.value
  if (v === null || v === undefined) return '—'
  if (Array.isArray(v)) {
    return v.map((x) => (typeof x === 'number' ? x.toFixed(3) : x)).join(', ')
  }
  if (typeof v === 'number') return v.toFixed(2)
  return String(v)
}

function fmtDetail(e) {
  const d = e.latest && e.latest.detail
  if (!d) return '—'
  if (typeof d === 'string') return d
  // 挑几个关键字段展示
  const keys = Object.keys(d)
  return keys.slice(0, 4).map((k) => `${k}=${typeof d[k] === 'number' ? d[k].toFixed(2) : d[k]}`).join('  ')
}
</script>

<template>
  <div v-for="g in GROUP_ORDER" :key="g" class="group-title">{{ g }}</div>
  <template v-for="g in GROUP_ORDER" :key="g + '-cards'">
    <div v-for="c in groups[g]" :key="c.topic" class="card" :class="stateClass(c)">
      <div class="card-head">
        <span class="card-label">{{ c.label }}</span>
        <span class="card-state">{{ stateText(c) }}</span>
      </div>
      <div class="card-value">{{ fmtValue(c) }}</div>
      <div class="card-detail">{{ c.topic }}</div>
      <div class="card-detail">{{ fmtDetail(c) }}</div>
    </div>
  </template>
</template>
