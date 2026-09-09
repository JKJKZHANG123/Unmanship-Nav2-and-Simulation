<script setup>
import { ref, onErrorCaptured } from 'vue'

const props = defineProps({
  name: { type: String, default: '模块' },
})

const error = ref(null)

// 捕获任何后代组件的渲染/生命周期/事件处理错误，
// 只在本区块显示错误提示，阻止错误继续向上冒泡导致整页卸载。
onErrorCaptured((err, instance, info) => {
  error.value = err && err.message ? err.message : String(err)
  console.error(`[ErrorBoundary:${props.name}]`, err, info)
  // 返回 false 阻止错误继续冒泡
  return false
})

function retry() {
  error.value = null
}
</script>

<template>
  <div v-if="error" class="error-boundary">
    <div class="eb-title">⚠ {{ name }} 加载出错</div>
    <div class="eb-msg">{{ error }}</div>
    <button class="btn btn-ghost" @click="retry">重试</button>
  </div>
  <slot v-else />
</template>
