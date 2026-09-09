<script setup>
import { ref, reactive } from 'vue'
import { PARAM_GROUPS, ALL_PARAMS, fullParamName, toParamString, fromParamString } from '../params.js'
import FootprintEditor from './FootprintEditor.vue'

const props = defineProps({
  client: { type: Object, required: true },
  connected: { type: Boolean, default: false },
})

// footprint 画板 ref，读取参数后同步
const fpEditor = ref(null)

// 每个参数的实时状态：{ loaded, value, error, saving, justSaved }
const paramState = reactive({})
const loading = ref(false)
const restoring = ref(false)
const notice = ref('')

function initState() {
  for (const p of ALL_PARAMS) {
    const key = fullParamName(p)
    const def = p.type === 'bool' ? (p.default ? 'true' : 'false') : String(p.default)
    // 初始显示「出厂默认值」；用户点「读取」后才会被运行时真实值覆盖。
    paramState[key] = {
      loaded: false,
      value: def,
      default: def,          // 出厂默认值（对比基准）
      isModified: false,     // 当前值是否已偏离默认值
      error: '',
      saving: false,
      justSaved: false,
    }
  }
}
initState()

function findParam(key) {
  return ALL_PARAMS.find((p) => fullParamName(p) === key)
}

async function readAll() {
  if (!props.connected) { notice.value = '未连接，无法读取参数'; return }
  loading.value = true
  notice.value = ''
  let ok = 0
  for (const p of ALL_PARAMS) {
    const key = fullParamName(p)
    const st = paramState[key]
    st.error = ''
    try {
      const res = await props.client.callService('/rosapi/get_param', { name: key, default_value: '' })
      if (res && res.successful) {
        st.value = fromParamString(p.type, res.value)
        st.loaded = true
        st.isModified = String(st.value) !== String(st.default)
        ok++
        if (p.name === 'footprint' && fpEditor.value) {
          fpEditor.value.syncFromString(String(st.value))
        }
      } else {
        st.error = (res && res.reason) || '读取失败'
      }
    } catch (e) {
      st.error = e.message
    }
  }
  loading.value = false
  notice.value = ok > 0 ? `已读取 ${ok}/${ALL_PARAMS.length} 个参数` : '读取失败（节点未运行或路径不对）'
}

async function writeParam(p, node) {
  const key = fullParamName(p, node)
  const st = paramState[key]
  st.saving = true
  st.error = ''
  st.justSaved = false
  try {
    const res = await props.client.callService('/rosapi/set_param', {
      name: key,
      value: toParamString(p.type, st.value),
    })
    if (res && res.successful) {
      st.justSaved = true
      st.isModified = String(st.value) !== String(st.default)
      notice.value = `已写入 ${p.name} = ${st.value}`
      setTimeout(() => { st.justSaved = false }, 1500)
    } else {
      st.error = (res && res.reason) || '写入失败'
    }
  } catch (e) {
    st.error = e.message
  }
  st.saving = false
}

function resetParam(p, node) {
  const key = fullParamName(p, node)
  paramState[key].value = paramState[key].default
  paramState[key].error = ''
  paramState[key].justSaved = false
  paramState[key].isModified = false
  // 只回填输入框为默认值，不自动写入（用户确认后再点「写入」）
  if (p.name === 'footprint' && fpEditor.value) {
    fpEditor.value.syncFromString(paramState[key].default)
  }
}

async function resetAllToDefault() {
  if (!props.connected) { notice.value = '未连接，无法写入'; return }
  if (!confirm('将把所有参数写回出厂默认值（等价于重启节点后的配置）。确定？')) return
  restoring.value = true
  notice.value = ''
  let ok = 0, fail = 0
  for (const p of ALL_PARAMS) {
    const key = fullParamName(p)
    const st = paramState[key]
    st.error = ''
    const def = p.type === 'bool' ? (p.default ? 'true' : 'false') : String(p.default)
    st.value = def
    try {
      const res = await props.client.callService('/rosapi/set_param', {
        name: key,
        value: toParamString(p.type, def),
      })
      if (res && res.successful) { ok++ } else { fail++; st.error = (res && res.reason) || '失败' }
    } catch (e) { fail++; st.error = e.message }
  }
  restoring.value = false
  notice.value = `恢复出厂完成：成功 ${ok}，失败 ${fail}`
}

// 数值输入合法性提示
function valueHint(p, node) {
  const st = paramState[fullParamName(p, node)]
  if (p.type !== 'number' || !st || st.value === '') return ''
  const v = Number(st.value)
  if (isNaN(v)) return '请输入数字'
  if (p.min !== undefined && v < p.min) return `最小 ${p.min}`
  if (p.max !== undefined && v > p.max) return `最大 ${p.max}`
  return ''
}
</script>

<template>
  <div class="param-panel">
    <div class="param-toolbar">
      <button class="btn" :disabled="loading || !connected" @click="readAll">
        {{ loading ? '读取中…' : '读取当前参数' }}
      </button>
      <button class="btn btn-reset" :disabled="restoring || !connected" @click="resetAllToDefault">
        {{ restoring ? '恢复中…' : '恢复全部出厂设置' }}
      </button>
      <span class="notice" :class="{ error: notice.includes('失败') || notice.includes('未连接') }">{{ notice }}</span>
    </div>

    <div v-for="g in PARAM_GROUPS" :key="g.title" class="param-group">
      <div class="param-group-title">{{ g.title }} <span class="param-node">{{ g.node }}</span></div>

      <template v-for="p in g.params" :key="g.node + p.name">
      <!-- footprint 用多边形画板编辑 -->
      <div v-if="p.name === 'footprint'" class="param-row param-row-wide">
        <div class="param-meta">
          <div class="param-name">
            {{ p.name }} <span class="param-unit">多边形</span>
            <span v-if="paramState[fullParamName(p, g.node)].isModified" class="param-modified">已修改</span>
          </div>
          <div class="param-desc">{{ p.desc }}</div>
          <div v-if="paramState[fullParamName(p, g.node)].loaded" class="param-values">
            <span class="pv-cur">当前 {{ paramState[fullParamName(p, g.node)].value }}</span>
            <span class="pv-def">默认 {{ paramState[fullParamName(p, g.node)].default }}</span>
          </div>
        </div>
        <FootprintEditor
          ref="fpEditor"
          :model-value="paramState[fullParamName(p, g.node)].value"
          @update:model-value="(v) => { paramState[fullParamName(p, g.node)].value = v }"
        />
        <div class="param-ctrl">
          <button
            class="btn btn-send"
            :disabled="paramState[fullParamName(p, g.node)].saving || !connected"
            @click="writeParam(p, g.node)"
          >
            {{ paramState[fullParamName(p, g.node)].saving ? '…' : paramState[fullParamName(p, g.node)].justSaved ? '✓ 已发送' : '发送 footprint' }}
          </button>
          <button class="btn btn-ghost" @click="resetParam(p, g.node)" title="恢复该参数的出厂默认值">↺ 恢复</button>
        </div>
        <div v-if="paramState[fullParamName(p, g.node)].error" class="param-error">
          {{ paramState[fullParamName(p, g.node)].error }}
        </div>
      </div>

      <!-- 其它参数用输入框 -->
      <div v-else class="param-row">
        <div class="param-meta">
          <div class="param-name">
            {{ p.name }}
            <span v-if="p.unit" class="param-unit">{{ p.unit }}</span>
            <span v-if="paramState[fullParamName(p, g.node)].isModified" class="param-modified">已修改</span>
          </div>
          <div class="param-desc">{{ p.desc }}</div>
          <div v-if="p.type === 'number' && p.min !== undefined && p.max !== undefined" class="param-range">
            范围 {{ p.min }} ~ {{ p.max }}
          </div>
          <div v-if="paramState[fullParamName(p, g.node)].loaded" class="param-values">
            <span class="pv-def">默认 {{ paramState[fullParamName(p, g.node)].default }}</span>
          </div>
        </div>

        <div class="param-ctrl">
          <input
            v-if="p.type === 'number'"
            v-model="paramState[fullParamName(p, g.node)].value"
            type="number"
            class="param-input"
            :step="p.step || 1"
            :min="p.min"
            :max="p.max"
            :class="{ 'has-error': paramState[fullParamName(p, g.node)].error || valueHint(p, g.node) }"
          />
          <input
            v-else-if="p.type === 'string'"
            v-model="paramState[fullParamName(p, g.node)].value"
            type="text"
            class="param-input mono"
            :class="{ 'has-error': paramState[fullParamName(p, g.node)].error }"
          />
          <select
            v-else
            v-model="paramState[fullParamName(p, g.node)].value"
            class="param-input"
          >
            <option value="true">开启</option>
            <option value="false">关闭</option>
          </select>

          <button
            class="btn btn-send"
            :disabled="paramState[fullParamName(p, g.node)].saving || !connected || !!valueHint(p, g.node)"
            @click="writeParam(p, g.node)"
          >
            {{ paramState[fullParamName(p, g.node)].saving ? '…' : paramState[fullParamName(p, g.node)].justSaved ? '✓ 已发送' : '发送' }}
          </button>
          <button class="btn btn-ghost" @click="resetParam(p, g.node)" title="恢复该参数的出厂默认值">↺</button>
        </div>

        <div v-if="valueHint(p, g.node)" class="param-error">{{ valueHint(p, g.node) }}</div>
        <div v-else-if="paramState[fullParamName(p, g.node)].error" class="param-error">
          {{ paramState[fullParamName(p, g.node)].error }}
        </div>
      </div>
      </template>
    </div>
  </div>
</template>
