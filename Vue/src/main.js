import { createApp } from 'vue'
import App from './App.vue'
import './style.css'

const app = createApp(App)

// 全局兜底错误处理：任何未捕获的组件错误都记录到 console，但不让整页崩溃。
// 具体区块的错误由 <ErrorBoundary> 局部捕获并展示。
app.config.errorHandler = (err, instance, info) => {
  console.error('[Vue errorHandler]', err, info, instance)
}
app.config.warnHandler = (msg, instance, trace) => {
  // 保留警告日志便于调试
  console.warn('[Vue warn]', msg, trace)
}

app.mount('#app')
