<script setup>
import { ref, reactive, onMounted, onBeforeUnmount } from 'vue'
import { RosbridgeClient } from './rosbridge.js'
import { createStore } from './store.js'
import { TOPICS } from './topics.js'
import TopoGraph from './components/TopoGraph.vue'
import DataCards from './components/DataCards.vue'
import TrendChart from './components/TrendChart.vue'
import TrajectoryCompare from './components/TrajectoryCompare.vue'
import ParamPanel from './components/ParamPanel.vue'
import ErrorBoundary from './components/ErrorBoundary.vue'

const WS_URL = (window.location.protocol === 'https:' ? 'wss://' : 'ws://') +
  window.location.host + '/rosbridge'

const store = createStore()
const conn = reactive({ status: 'disconnected', text: '未连接' })

const client = new RosbridgeClient(WS_URL)
client.setTopicTypes(Object.fromEntries(TOPICS.map((t) => [t.topic, t.type])))
client.onStatus = (status, detail) => {
  conn.status = status
  conn.text = status === 'connected' ? `已连接 ${detail}` : '已断开，重连中…'
  store.stats.connected = status === 'connected'
}

onMounted(() => {
  for (const t of TOPICS) {
    client.subscribe(t.topic, (msg) => store.onMessage(t.topic, msg))
  }
  client.connect()
})

onBeforeUnmount(() => {
  client.close()
})
</script>

<template>
  <header class="app-header">
    <div class="app-title">USV 数据监控台 <span class="dot">·</span> 实船链路</div>
    <div class="conn-badge" :class="{ connected: conn.status === 'connected' }">
      <span class="led"></span>
      {{ conn.text }}
    </div>
  </header>

  <section class="stat-strip">
    <div class="stat-chip">监控话题 <b>{{ store.stats.total }}</b></div>
    <div class="stat-chip ok">正常 <b>{{ store.stats.healthyCount }}</b></div>
    <div class="stat-chip warn">断流/超时 <b>{{ store.stats.staleCount }}</b></div>
    <div class="stat-chip err">解析异常 <b>{{ store.stats.errorCount }}</b></div>
  </section>

  <main class="main-grid">
    <!-- 拓扑图 -->
    <section class="panel full">
      <div class="panel-title">数据流向拓扑（绿=正常 · 红=解析异常 · 灰=无数据/超时）</div>
      <ErrorBoundary name="数据流向拓扑">
        <TopoGraph :topics="store.topics" />
      </ErrorBoundary>
    </section>

    <!-- 轨迹对比 -->
    <section class="panel">
      <div class="panel-title">GPS vs LIO 轨迹对比</div>
      <ErrorBoundary name="轨迹对比">
        <TrajectoryCompare :trajectory="store.trajectory" />
      </ErrorBoundary>
    </section>

    <!-- 实时趋势 -->
    <section class="panel">
      <div class="panel-title">实时趋势</div>
      <ErrorBoundary name="实时趋势">
        <TrendChart :topics="store.topics" />
      </ErrorBoundary>
    </section>

    <!-- 参数读写（左列） -->
    <section class="panel">
      <div class="panel-title">参数读写（读取当前参数 → 修改 → 写入）</div>
      <ErrorBoundary name="参数读写">
        <ParamPanel :client="client" :connected="store.stats.connected" />
      </ErrorBoundary>
    </section>

    <!-- 数据卡片（右列） -->
    <section class="panel">
      <div class="panel-title">话题数据卡片</div>
      <div class="cards cards-single">
        <ErrorBoundary name="数据卡片">
          <DataCards :topics="store.topics" />
        </ErrorBoundary>
      </div>
    </section>
  </main>
</template>
