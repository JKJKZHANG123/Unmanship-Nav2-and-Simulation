<script setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'

const props = defineProps({
  modelValue: { type: String, default: '[[0,0],[1,0],[1,1],[0,1]]' },
})

const emit = defineEmits(['update:modelValue'])

// 画布尺寸（像素）
const W = 460
const H = 320
const PAD = 40

const svg = ref(null)
const dragging = ref(-1)  // 正在拖动的顶点索引，-1 表示无

// 解析 footprint 字符串 -> 点数组 [{x,y}]
function parseFootprint(str) {
  try {
    const arr = JSON.parse(str)
    if (Array.isArray(arr) && arr.length >= 3) {
      return arr
        .map((pt) => ({ x: Number(pt[0]), y: Number(pt[1]) }))
        .filter((pt) => isFinite(pt.x) && isFinite(pt.y))
    }
  } catch (e) { /* ignore */ }
  return null
}

const points = ref(parseFootprint(props.modelValue) || [{ x: -1, y: -0.5 }, { x: 1, y: -0.5 }, { x: 1, y: 0.5 }, { x: -1, y: 0.5 }])

// 计算缩放与偏移，把米制坐标映射到画布像素
const transform = computed(() => {
  const xs = points.value.map((p) => p.x)
  const ys = points.value.map((p) => p.y)
  const minX = Math.min(...xs), maxX = Math.max(...xs)
  const minY = Math.min(...ys), maxY = Math.max(...ys)
  const spanX = Math.max(maxX - minX, 0.01)
  const spanY = Math.max(maxY - minY, 0.01)
  const scale = Math.min((W - PAD * 2) / spanX, (H - PAD * 2) / spanY)
  const cx = (minX + maxX) / 2
  const cy = (minY + maxY) / 2
  return { scale, cx, cy }
})

function toPx(p) {
  const t = transform.value
  return { x: W / 2 + (p.x - t.cx) * t.scale, y: H / 2 - (p.y - t.cy) * t.scale }
}
function toMeter(px, py) {
  const t = transform.value
  return { x: (px - W / 2) / t.scale + t.cx, y: -(py - H / 2) / t.scale + t.cy }
}

const polygonPoints = computed(() =>
  points.value.map((p) => {
    const q = toPx(p)
    return `${q.x.toFixed(1)},${q.y.toFixed(1)}`
  }).join(' ')
)

const vertices = computed(() =>
  points.value.map((p) => toPx(p))
)

// 序列化回 footprint 字符串
function serialize() {
  const s = points.value.map((p) => `[${p.x.toFixed(3)}, ${p.y.toFixed(3)}]`).join(', ')
  return `[${s}]`
}

function onMouseDown(i, ev) {
  dragging.value = i
  ev.preventDefault()
}

function onMouseMove(ev) {
  if (dragging.value < 0) return
  const rect = svg.value.getBoundingClientRect()
  const scaleX = W / rect.width
  const scaleY = H / rect.height
  const px = (ev.clientX - rect.left) * scaleX
  const py = (ev.clientY - rect.top) * scaleY
  const m = toMeter(px, py)
  points.value[dragging.value] = { x: m.x, y: m.y }
  emit('update:modelValue', serialize())
}

function onMouseUp() {
  dragging.value = -1
}

// 加一个顶点（在最后一条边的中点）
function addVertex() {
  if (points.value.length < 2) return
  const a = points.value[points.value.length - 1]
  const b = points.value[0]
  points.value.splice(points.value.length, 0, {
    x: (a.x + b.x) / 2,
    y: (a.y + b.y) / 2,
  })
  emit('update:modelValue', serialize())
}

// 删除顶点（至少保留 3 个）
function removeVertex(i) {
  if (points.value.length <= 3) return
  points.value.splice(i, 1)
  emit('update:modelValue', serialize())
}

onMounted(() => {
  window.addEventListener('mousemove', onMouseMove)
  window.addEventListener('mouseup', onMouseUp)
})
onBeforeUnmount(() => {
  window.removeEventListener('mousemove', onMouseMove)
  window.removeEventListener('mouseup', onMouseUp)
})

// 外部（读取参数后）同步进来
function syncFromString(str) {
  const parsed = parseFootprint(str)
  if (parsed) points.value = parsed
}
defineExpose({ syncFromString })
</script>

<template>
  <div class="fp-editor">
    <svg
      ref="svg"
      :viewBox="`0 0 ${W} ${H}`"
      class="fp-svg"
    >
      <!-- 网格背景 -->
      <defs>
        <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
          <path d="M 20 0 L 0 0 0 20" fill="none" stroke="#1e293b" stroke-width="0.5" />
        </pattern>
      </defs>
      <rect :width="W" :height="H" fill="url(#grid)" />
      <!-- 船头方向箭头（x 正方向 = 船头） -->
      <line :x1="W/2" :y1="H/2" :x2="W/2+50" :y2="H/2" stroke="#38bdf8" stroke-width="1" stroke-dasharray="4 3" />
      <text :x="W/2+55" :y="H/2+4" fill="#38bdf8" font-size="11">船头 →</text>
      <!-- 多边形 -->
      <polygon :points="polygonPoints" fill="rgba(56,189,248,0.15)" stroke="#38bdf8" stroke-width="2" />
      <!-- 顶点 -->
      <g v-for="(v, i) in vertices" :key="i" style="cursor: grab;">
        <circle
          :cx="v.x" :cy="v.y" r="7"
          fill="#0b1220" stroke="#38bdf8" stroke-width="2"
          @mousedown="onMouseDown(i, $event)"
          @dblclick="removeVertex(i)"
        />
        <text :x="v.x + 10" :y="v.y - 6" fill="#e2e8f0" font-size="10">{{ i }}</text>
      </g>
    </svg>
    <div class="fp-toolbar">
      <span class="fp-hint">拖动圆点改变轮廓 · 双击圆点删除 · 船头朝右</span>
      <button class="btn btn-ghost" @click="addVertex">+ 添加顶点</button>
    </div>
  </div>
</template>
